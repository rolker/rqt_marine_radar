#include "rqt_marine_radar/marine_radar_plugin.h"
#include <QOpenGLShader>
#include <QOpenGLTexture>

namespace rqt_marine_radar
{
    
RadarWidget::RadarWidget(QWidget *parent): QOpenGLWidget(parent)
{
}

void RadarWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    
    QVector<GLfloat> vertData;
    vertData.append(-1.0); vertData.append(-1.0); vertData.append(0.0);
    vertData.append(1.0); vertData.append(-1.0); vertData.append(0.0);
    vertData.append(1.0); vertData.append(1.0); vertData.append(0.0);
    vertData.append(-1.0); vertData.append(1.0); vertData.append(0.0);
    
    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
    
#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
        "attribute highp vec4 vertex;\n"
        "varying mediump vec4 texc;\n"
        "uniform mediump mat4 matrix;\n"
        "void main(void)\n"
        "{\n"
        "    gl_Position = matrix * vertex;\n"
        "    texc = vertex;\n"
        "}\n";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
        "#define M_PI 3.1415926535897932384626433832795\n"
        "uniform sampler2D texture;\n"
        "varying mediump vec4 texc;\n"
        "uniform float minAngle;\n"
        "uniform float maxAngle;\n"
        "void main(void)\n"
        "{\n"
        "    if(texc.x == 0.0) discard;\n"
        "    float r = length(texc.xy);\n"
        "    if(r>1.0) discard;\n"
        "    float theta = atan(texc.y, texc.x);\n"
        "    if(minAngle > 0.0 && theta < 0.0) theta += 2.0*M_PI;\n"
        "    if(theta < minAngle) discard;\n"
        "    if(theta > maxAngle) discard;\n"
        "    gl_FragColor = texture2D(texture, vec2(r, (theta-minAngle)/(maxAngle-minAngle)));\n"
        "}\n";
    fshader->compileSourceCode(fsrc);

    m_program = new QOpenGLShaderProgram;
    m_program->addShader(vshader);
    m_program->addShader(fshader);
    m_program->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
    m_program->link();

    m_program->bind();
    m_program->setUniformValue("texture", 0);
}

void RadarWidget::resizeGL(int w, int h)
{
    m_matrix.setToIdentity();
    float wf = 1.0;
    float hf = 1.0;
    if (w > h)
        wf = w/float(h);
    if (h > w)
        hf = h/float(w);
    m_matrix.ortho(-wf, +wf, -hf, +hf, 4.0f, 15.0f);
    m_matrix.translate(0.0f, 0.0f, -10.0f);
}

void RadarWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);

    m_program->setUniformValue("matrix", m_matrix);
    m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
    m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 3 * sizeof(GLfloat));
    
    while(m_sectors.size() > 75)
    {
        if(m_sectors.front().sectorImage)
            delete m_sectors.front().sectorImage;
        if(m_sectors.front().sectorTexture)
            delete m_sectors.front().sectorTexture;
        m_sectors.pop_front();
    }
    for(Sector &s: m_sectors)
        if(s.sectorImage)
        {
            if(!s.sectorTexture)
            {
                s.sectorTexture = new QOpenGLTexture(*s.sectorImage);
            }
            m_program->setUniformValue("minAngle", GLfloat(s.angle2-s.half_scanline_angle*1.1));
            m_program->setUniformValue("maxAngle", GLfloat(s.angle1+s.half_scanline_angle*1.1));
            s.sectorTexture->bind();
            glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        }
    
}

void RadarWidget::addSector(double angle1, double angle2, double range, QImage *sector)
{
    //std::cerr << angle1 << " - " << angle2 << " rads, " << range << " meters" << std::endl;
    Sector s;
    if(angle1 < angle2)
        s.angle1 = angle1+(2.0*M_PI);
    else
        s.angle1 = angle1;
    s.angle2 = angle2;
    s.half_scanline_angle = (s.angle1 - s.angle2)/(2.0*sector->height());
    s.range = range;
    s.sectorImage = sector;
    m_sectors.push_back(s);
    update();
}

} //namespace
