#include "rqt_marine_radar/marine_radar_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <marine_radar_control_msgs/RadarControlValue.h>
#include <QLineEdit>
#include <QLabel>

namespace rqt_marine_radar
{
    
MarineRadarPlugin::MarineRadarPlugin():rqt_gui_cpp::Plugin(), m_widget(0)
{
    setObjectName("MarineRadar");
}
 
void MarineRadarPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    m_widget = new QWidget();
    m_ui.setupUi(m_widget);
    
    m_widget->setWindowTitle(m_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    
    context.addWidget(m_widget);
    
    updateTopicList();
    m_ui.topicsComboBox->setCurrentIndex(m_ui.topicsComboBox->findText(""));
    connect(m_ui.topicsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

    m_ui.refreshTopicsPushButton->setIcon(QIcon::fromTheme("view-refresh"));
    connect(m_ui.refreshTopicsPushButton, SIGNAL(pressed()), this, SLOT(updateTopicList()));
    
    connect(m_ui.showControlsPushButton, SIGNAL(pressed()), this, SLOT(onShowControlsPushButtonClicked()));
    connect(m_ui.showRadarPushButton, SIGNAL(pressed()), this, SLOT(onShowRadarPushButtonClicked()));
    connect(m_ui.fadeTimeDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onFadePeriodDoubleSpinBoxValueChanged()));

    // set topic name if passed in as argument
    const QStringList& argv = context.argv();
    if (!argv.empty()) {
        m_arg_topic = argv[0];
        selectTopic(m_arg_topic);
    }
    
}

void MarineRadarPlugin::shutdownPlugin()
{
    {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        for(auto c: m_connections)
            QObject::disconnect(c);
    }
    m_dataSubscriber.shutdown();
    m_stateSubscriber.shutdown();
    m_stateChangePublisher.shutdown();
}

void MarineRadarPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    QString topic = m_ui.topicsComboBox->currentText();
    instance_settings.setValue("topic", topic);
}

void MarineRadarPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString topic = instance_settings.value("topic", "").toString();
    // don't overwrite topic name passed as command line argument
    if (!m_arg_topic.isEmpty())
    {
        m_arg_topic = "";
    }
    else
    {
        selectTopic(topic);
    }
}

void MarineRadarPlugin::updateTopicList()
{
    QString selected = m_ui.topicsComboBox->currentText();
    
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    
    QList<QString> topics;
    for(const auto t: topic_info)
        if (t.datatype == "marine_radar_control_msgs/RadarControlSet")
            topics.append(t.name.c_str());
        
    topics.append("");
    qSort(topics);
    m_ui.topicsComboBox->clear();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
        QString label(*it);
        label.replace(" ", "/");
        m_ui.topicsComboBox->addItem(label, QVariant(*it));
    }

    // restore previous selection
    selectTopic(selected);
}

void MarineRadarPlugin::selectTopic(const QString& topic)
{
    int index = m_ui.topicsComboBox->findText(topic);
    if (index == -1)
    {
        // add topic name to list if not yet in
        QString label(topic);
        label.replace(" ", "/");
        m_ui.topicsComboBox->addItem(label, QVariant(topic));
        index = m_ui.topicsComboBox->findText(topic);
    }
    m_ui.topicsComboBox->setCurrentIndex(index);
}

void MarineRadarPlugin::onTopicChanged(int index)
{
    m_dataSubscriber.shutdown();
    m_stateSubscriber.shutdown();
    m_stateChangePublisher.shutdown();
    
    QString topic = m_ui.topicsComboBox->itemData(index).toString();
    if(!topic.isEmpty())
    {
        m_stateSubscriber = getNodeHandle().subscribe(topic.toStdString(), 10, &MarineRadarPlugin::stateCallback, this);

        QString data_topic = topic;
        data_topic.chop(5);
        data_topic += "data";
        
        m_dataSubscriber = getNodeHandle().subscribe(data_topic.toStdString(), 10, &MarineRadarPlugin::dataCallback, this);

        QString state_change_topic = topic;
        state_change_topic.chop(5);
        state_change_topic += "change_state";
        
        m_stateChangePublisher = getNodeHandle().advertise<marine_radar_control_msgs::RadarControlValue>(state_change_topic.toStdString(),10);
    }
}

void MarineRadarPlugin::onShowControlsPushButtonClicked()
{
    m_ui.scrollArea->setVisible(!m_ui.scrollArea->isVisible());
}

void MarineRadarPlugin::onShowRadarPushButtonClicked()
{
    m_ui.openGLWidget->setVisible(!m_ui.openGLWidget->isVisible());
}

void MarineRadarPlugin::onFadePeriodDoubleSpinBoxValueChanged()
{
    m_ui.openGLWidget->setFadeTime(m_ui.fadeTimeDoubleSpinBox->value());
}


void MarineRadarPlugin::dataCallback(const marine_sensor_msgs::RadarSector& msg)
{
    //std::cerr << "radar data!" << std::endl;
    if (!msg.intensities.empty())
    {
        double angle1 = msg.angle_start;
        double angle2 = angle1+ msg.angle_increment*(msg.intensities.size()-1);
        double range = msg.range_max;
        int w = msg.intensities.front().echoes.size();
        int h = msg.intensities.size();
        QImage * sector = new QImage(w,h,QImage::Format_Grayscale8);
        sector->fill(Qt::darkGray);
        for(int i = 0; i < h; i++)
            for(int j = 0; j < w; j++)
                sector->bits()[(h-1-i)*w+j] = msg.intensities[i].echoes[j]*255; // convert from float to 8 bits
        QDateTime timestamp = QDateTime::fromMSecsSinceEpoch(msg.header.stamp.toSec()*1000,Qt::UTC);
        QMetaObject::invokeMethod(m_ui.openGLWidget,"addSector", Qt::QueuedConnection, Q_ARG(double, angle1), Q_ARG(double, angle2), Q_ARG(double, range), Q_ARG(QImage *, sector), Q_ARG(QDateTime, timestamp));
    }
}

void MarineRadarPlugin::stateCallback(const marine_radar_control_msgs::RadarControlSet& msg)
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_new_state.clear();
    for(const auto i: msg.items)
        m_new_state.push_back(i);
    
    QMetaObject::invokeMethod(this,"updateState", Qt::QueuedConnection);
}

void MarineRadarPlugin::updateState()
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    for(auto state: m_new_state)
    {
        //std::cerr << state.name << ": " << state.value << std::endl;
        if(m_controls.find(state.name) == m_controls.end())
        {
            int insertRow = m_ui.controlsGridLayout->rowCount();
            ControlSet cs;
            cs.label = new QLabel(QString::fromStdString(state.label));
            cs.state = new QLabel(QString::fromStdString(state.value));
            switch (state.type)
            {
                case marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_FLOAT:
                    {
                        QLineEdit *le = new QLineEdit();
                        le->setMaximumWidth(100); 
                        QDoubleValidator *v = new QDoubleValidator(le);
                        if (state.max_value > state.min_value)
                            v->setRange(state.min_value, state.max_value, 2);
                        le->setValidator(v);
                        le->setToolTip("Range: " + QString::number(state.min_value) + " to " + QString::number(state.max_value));
                        cs.input = le;
                        m_connections.push_back(connect(le, &QLineEdit::editingFinished, this, [=](){marine_radar_control_msgs::RadarControlValue kv; kv.key=state.name; kv.value=le->text().toStdString(); this->m_stateChangePublisher.publish(kv);}));
                    }
                    break;
                case marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_FLOAT_WITH_AUTO:
                    {
                        cs.input = new QWidget();
                        QHBoxLayout *horizontalLayout = new QHBoxLayout(cs.input);
                        horizontalLayout->setContentsMargins(0,0,0,0);
                        QLineEdit *lineEdit = new QLineEdit();
                        lineEdit->setMaximumWidth(60);
                        QDoubleValidator *v = new QDoubleValidator(lineEdit);
                        if (state.max_value > state.min_value)
                            v->setRange(state.min_value, state.max_value);
                        lineEdit->setValidator(v);
                        lineEdit->setToolTip("Range: " + QString::number(state.min_value) + " to " + QString::number(state.max_value));

                        m_connections.push_back(connect(lineEdit, &QLineEdit::editingFinished, this, [=](){marine_radar_control_msgs::RadarControlValue kv; kv.key=state.name; kv.value=lineEdit->text().toStdString(); this->m_stateChangePublisher.publish(kv);}));
                        horizontalLayout->addWidget(lineEdit);
                        QPushButton *autoButton = new QPushButton("auto");
                        autoButton->setMaximumWidth(35);
                        horizontalLayout->addWidget(autoButton);
                        m_connections.push_back(connect(autoButton, &QAbstractButton::clicked, this, [=](){marine_radar_control_msgs::RadarControlValue kv; kv.key=state.name; kv.value="auto"; this->m_stateChangePublisher.publish(kv);}));
                    }
                    break;
                case marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_ENUM:
                    {
                        QComboBox *cb = new QComboBox();
                        for(auto e: state.enums)
                            cb->addItem(QString::fromStdString(e));
                        cb->setMaximumWidth(100); 
                        m_connections.push_back(connect(cb, QOverload<int>::of(&QComboBox::activated), this, [=](int index){marine_radar_control_msgs::RadarControlValue kv; kv.key=state.name; kv.value=cb->itemText(index).toStdString(); this->m_stateChangePublisher.publish(kv);}));
                        cs.input = cb;
                    }
                    break;
            }       
            m_ui.controlsGridLayout->addWidget(cs.label,insertRow,0);
            m_ui.controlsGridLayout->addWidget(cs.state,insertRow,1);
            m_ui.controlsGridLayout->addWidget(cs.input,insertRow,2);
            m_controls[state.name] = cs;
        }
        else
        {
            m_controls[state.name].state->setText(QString::fromStdString(state.value));
        }
    }
}

}

PLUGINLIB_EXPORT_CLASS(rqt_marine_radar::MarineRadarPlugin, rqt_gui_cpp::Plugin)
