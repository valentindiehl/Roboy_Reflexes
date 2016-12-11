#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDir>
#include <QProcessEnvironment>
#include <map>
#include <vector>
#include <ros/ros.h>

using namespace std;

struct Trajectory{
    int control_mode; // position, velocity, force
    vector<float> setPoints;
};

struct Behaviour{
    QString name;
    int id;
    int samplerate; // in ms
    map<int,Trajectory> trajectories; // motor/trajectory pair
};

class RoboyBehaviourXmlParser
{
public:
    RoboyBehaviourXmlParser();
    QString roboyBehaviourDir;
    bool readRoboyBehavior( QString name, Behaviour &behaviour );
    bool readTrajectories(Behaviour &behavior);
private:
    QString m_databasePath;
    QXmlStreamReader m_xmlReader;
    QXmlStreamWriter  m_xmlWriter;

};
