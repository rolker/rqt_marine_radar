#ifndef RQT_MARINE_RADAR_MARINE_RADAR_PLUGIN_H
#define RQT_MARINE_RADAR_MARINE_RADAR_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_marine_radar_plugin.h>
#include <ros/ros.h>
#include <marine_sensor_msgs/RadarSector.h>
#include <marine_sensor_msgs/RadarControlSet.h>
#include <mutex>

class QLabel;

namespace rqt_marine_radar
{
    
class MarineRadarPlugin: public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    MarineRadarPlugin();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
protected slots:
    virtual void updateTopicList();
    
    virtual void selectTopic(const QString& topic);  

    virtual void onTopicChanged(int index);
    virtual void onShowControlsPushButtonClicked();
    virtual void onShowRadarPushButtonClicked();
  
    virtual void dataCallback(const marine_sensor_msgs::RadarSector &msg);
    virtual void stateCallback(const marine_sensor_msgs::RadarControlSet &msg); 
  
    void updateState();
  
private:
    Ui::MarineRadarWidget m_ui;
    QWidget* m_widget;
    ros::Subscriber m_dataSubscriber;
    ros::Subscriber m_stateSubscriber;
    ros::Publisher m_stateChangePublisher;

    QString m_arg_topic;
    
    struct ControlSet
    {
        QLabel *label;
        QLabel *state;
        QWidget *input;
    };
    
    std::map<std::string,ControlSet> m_controls;
    std::vector<QMetaObject::Connection> m_connections;
    
    std::vector<marine_sensor_msgs::RadarControlItem> m_new_state;
    std::mutex m_state_mutex;
};

} // namespace rqt_marine_radar

#endif
