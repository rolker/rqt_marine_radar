#ifndef RQT_MARINE_RADAR_RADAR_WIDGET_H
#define RQT_MARINE_RADAR_RADAR_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QDateTime>
#include <deque>

Q_DECLARE_METATYPE(QImage*)

class QOpenGLTexture;
class QOpenGLShaderProgram;


namespace rqt_marine_radar
{
    
class RadarWidget: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    RadarWidget(QWidget *parent);
    
public slots:
    void addSector(double angle1, double angle2, double range, QImage *sector, QDateTime timestamp);
    
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    
    struct Sector
    {
        Sector():angle1(0),angle2(0),range(0),sectorImage(nullptr),sectorTexture(nullptr)
        {}
        
        double angle1, angle2, range, half_scanline_angle;
        QImage *sectorImage;
        QOpenGLTexture *sectorTexture;
        QDateTime timestamp;
    };

    std::deque<Sector> m_sectors;
    
    QOpenGLShaderProgram *m_program;
    QOpenGLBuffer m_vbo;
    QMatrix4x4 m_matrix;
    double m_fade_time = 3.0;
    QTimer* m_update_timer = nullptr;
};
    
} // namespace

#endif

