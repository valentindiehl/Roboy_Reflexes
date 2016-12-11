#include "RoboyBehaviourXmlParser.hpp"

RoboyBehaviourXmlParser::RoboyBehaviourXmlParser() {
    roboyBehaviourDir = QProcessEnvironment::systemEnvironment().value("ROBOY_BEHAVIOUR_DIR");
    if (roboyBehaviourDir == "") {
        ROS_FATAL_STREAM("Environment Variable not set." <<
        "Set the environment variable ROBOY_BEHAVIOUR_DIR");
    }
}

bool RoboyBehaviourXmlParser::readRoboyBehavior( QString name, Behaviour &behaviour ) {
    ROS_INFO_STREAM( "READ BEHAVIOR " << name.toStdString());

    QString path = roboyBehaviourDir + "/" + name + ".xml";
    QFile file( path );

    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        ROS_ERROR_STREAM("Failed to open file: " << name.toStdString() + ".xml");
        return false;
    }

    m_xmlReader.setDevice(&file);

    while ( m_xmlReader.readNextStartElement() ) {
        if ( m_xmlReader.name() == "roboybehavior" &&
             m_xmlReader.attributes().hasAttribute("name") &&
             m_xmlReader.attributes().hasAttribute("behaviorid") ) {

            behaviour.name = m_xmlReader.attributes().value("name").toString();
            behaviour.id = m_xmlReader.attributes().value("behaviorid").toString().toULong();

            ROS_INFO_STREAM("Name:\t" << behaviour.name.toStdString());
            ROS_INFO_STREAM("Id:\t" << behaviour.id);
        }else if ( m_xmlReader.name() == "trajectory" ) {
            readTrajectories(behaviour);
            m_xmlReader.skipCurrentElement();
        } else {
            m_xmlReader.skipCurrentElement();
        }
    }

    ROS_INFO("Finished reading successfully");
    return true;
}

bool RoboyBehaviourXmlParser::readTrajectories(Behaviour &behaviour) {
    if ( m_xmlReader.name() == "trajectory" ) {
        quint32 motor_id = m_xmlReader.attributes().value("motorid").toString().toUInt();
        qint32 controlMode = m_xmlReader.attributes().value("controlmode").toString().toInt();
        qint32 sampleRate = m_xmlReader.attributes().value("samplerate").toString().toInt();

        Trajectory trajectory;
        trajectory.control_mode = controlMode;
        behaviour.samplerate = sampleRate;

        m_xmlReader.readNextStartElement();

        if(m_xmlReader.name() == "waypointlist") {
            QString valueList = m_xmlReader.readElementText();
            QChar separator(',');
            for (auto str : valueList.split(separator)) {
                trajectory.setPoints.push_back(str.toFloat());
            }
        }

        behaviour.trajectories[motor_id] = trajectory;

        ROS_INFO_STREAM("MOTOR ID: " << (int)motor_id <<
                  " SampleRate: " << (int)sampleRate <<
                  " ControlMode: " << (int)controlMode <<
                  " WAYPOINT COUNT: " << (int)behaviour.trajectories[motor_id].setPoints.size());

        return true;
    }

    return false;
}
