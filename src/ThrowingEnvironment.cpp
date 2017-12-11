#include "ThrowingEnvironment.h"
#include <cfloat>
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <stdexcept>
#include <cassert>


using namespace configmaps;

namespace bolero {
  namespace throwing_environment {

    ThrowingEnvironment::ThrowingEnvironment(lib_manager::LibManager *theManager)
      : Environment(theManager, "throwing_environment", 1.0),
        MARSEnvironment(theManager, "throwing_environment", 1.0),
        ContextualEnvironment(theManager, "throwing_environment", 1.0),
        startAnglePos(3),
        verbose(0),
        MAX_SIMULATION_TIME(2000),
        ballThrowTime(50),
        numJoints(3),
        armHeight(0.0),
        isSceneLoaded(false)
    {
      targetPos << 2.0, 0.0, 0.0;
      startAnglePos << 0.0, -0.8, -0.7;
    }

    ThrowingEnvironment::~ThrowingEnvironment()
    {
    }
  
    void ThrowingEnvironment::initMARSEnvironment()
    {
      readConfig();
      request_context(targetPos.data(), 2);

      if(!isSceneLoaded)
      {
        std::string sceneFile = getConfigPath() +
            "/throwing_environment/throwing.smurfs";
        control->sim->loadScene(sceneFile.c_str());
        isSceneLoaded = true;
      }

      getMotorIDs();
    }

    void ThrowingEnvironment::readConfig()
    {
      // Parameters of the environment are in the file "learning_config.yml".
      // It should be located in the current working directory. This
      // environment accepts the additional parameters
      // - ballThrowTime: after this has been reached (number of time steps),
      //   the ball will be detached from the robot
      // - armHeight: simulates that the arm is mounted on a table, this is the
      //   height of the table, the simulation stops when the ball hits the
      //   virtual ground
      // - verbose - verbosity level
      ConfigMap learningConfigMap = ConfigMap::fromYamlFile("learning_config.yml");
      if(learningConfigMap.find("Environment") != learningConfigMap.end())
      {
        if(learningConfigMap["Environment"].find("ballThrowTime") != learningConfigMap["Environment"].endMap())
          ballThrowTime = (learningConfigMap["Environment"])["ballThrowTime"];
        if(learningConfigMap["Environment"].find("armHeight") != learningConfigMap["Environment"].endMap())
          armHeight = learningConfigMap["Environment"]["armHeight"];
        if(learningConfigMap["Environment"].find("verbose") != learningConfigMap["Environment"].endMap())
          verbose = (learningConfigMap["Environment"])["verbose"];
      }

      if(verbose >= 2)
      {
        std::cout << "Configuration:" << std::endl
            << "    ballThrowTime = " << ballThrowTime << std::endl
            << "    armHeight = " << armHeight << std::endl
            << "    verbose = " << verbose << std::endl;
      }
    }

    std::string ThrowingEnvironment::getConfigPath()
    {
      // Here we use the environment variable "ROCK_CONFIGURATION_PATH" in
      // order to "find" the smurf file to be loaded. During installation it
      // should be put in "$ROCK_CONFIGURATION_PATH/spacebot_throw_environment".
      std::string configPath = std::string(getenv("ROCK_CONFIGURATION_PATH"));
      if(configPath.empty())
        throw std::runtime_error("WARNING: The ROCK_CONFIGURATION_PATH is not "
                                 "set! Did you \"source env.sh\"?\n");
      return configPath;
    }

    void ThrowingEnvironment::getMotorIDs()
    {
      motorIDs.clear();

      std::vector<mars::interfaces::core_objects_exchange>::iterator it;
      std::vector<mars::interfaces::core_objects_exchange> motorList;
      control->motors->getListMotors(&motorList);

      if(verbose >= 2)
      {
        if(motorList.empty())
          std::cout << "No motors available" << std::endl;
        else
          std::cout << "Available motors:" << std::endl;
        for(it = motorList.begin(); it != motorList.end(); ++it)
          std::cout << it->index << ": " << it->name << std::endl;
      }

      for(it = motorList.begin(); it != motorList.end(); ++it)
        motorIDs.push_back(it->index);
    }

    void ThrowingEnvironment::resetMARSEnvironment()
    {
      fitness = 0.0;
      evaluation_done = false;
      setStartAngles();
    }

    void ThrowingEnvironment::setStartAngles()
    {
      for(unsigned int i=0; i < motorIDs.size(); i++)
      {
        inputs[i] = startAnglePos(i);
        control->motors->setMotorValue(motorIDs[i], startAnglePos(i));
      }

      dataMutex.lock();
      handleInputValues();
      createOutputValues();
      dataMutex.unlock();
    }

    void ThrowingEnvironment::handleMARSError()
    {
      fitness = -DBL_MAX;
      evaluation_done = true;
    }

    int ThrowingEnvironment::getNumInputs() const
    {
      return numJoints;
    }

    int ThrowingEnvironment::getNumOutputs() const
    {
      return numJoints;
    }

    void ThrowingEnvironment::createOutputValues(void)
    {
      setPositionOfVisualTarget(); // must always be done, falls down otherwise
      outputMotorPositions();
      checkBallPosition();
      checkMaxTime();
    }

    void ThrowingEnvironment::setPositionOfVisualTarget()
    {
      mars::interfaces::NodeId targetId = control->nodes->getID("target_link");
      control->nodes->setPosition(targetId, targetPos);
    }

    void ThrowingEnvironment::outputMotorPositions()
    {
      for(unsigned int i = 0; i < motorIDs.size(); i++)
        outputs[i] = (double)control->motors->getActualPosition(motorIDs[i]);
    }

    void ThrowingEnvironment::checkBallPosition()
    {
      mars::interfaces::NodeId ballId = control->nodes->getID("ball_link");
      mars::utils::Vector ballPos = control->nodes->getPosition(ballId);

      if(ballPos[2] <= -armHeight)
      {
        ballHitX = ballPos[0];
        ballHitY = ballPos[1];
        const double diffX = ballPos[0] - targetPos[0];
        const double diffY = ballPos[1] - targetPos[1];
        const double squaredDist = diffX * diffX + diffY * diffY;
        fitness = -squaredDist;
        evaluation_done = true;
      }
    }

    void ThrowingEnvironment::checkMaxTime()
    {
      if(leftTime > MAX_SIMULATION_TIME) {
        fitness = DBL_MAX;
        evaluation_done = true;
      }
    }

    void ThrowingEnvironment::handleInputValues()
    {
      setMotorValues();
      checkBallThrowTime();
    }

    void ThrowingEnvironment::setMotorValues()
    {
      for(unsigned int i=0; i < motorIDs.size(); i++)
        control->motors->setMotorValue(motorIDs[i], inputs[i]);
    }

    void ThrowingEnvironment::checkBallThrowTime()
    {
      if(leftTime > ballThrowTime)
        control->joints->removeJoint(control->joints->getID("ball_joint"));
    }

    double* ThrowingEnvironment::request_context(double *context, int size)
    {
      if(size != 2)
        return NULL;

      targetPos[0] = context[0];
      targetPos[1] = context[1];
      targetPos[2] = -armHeight;

      return targetPos.data();
    }

    int ThrowingEnvironment::get_num_context_dims() const
    {
      return 2;
    }

    bool ThrowingEnvironment::isEvaluationDone() const
    {
      return evaluation_done;
    }

    int ThrowingEnvironment::getFeedback(double *feedback) const
    {
      feedback[0] = fitness;
      feedback[1] = ballHitX;
      feedback[2] = ballHitY;
      return 3;
    }

  }
}
DESTROY_LIB(bolero::throwing_environment::ThrowingEnvironment);
CREATE_LIB(bolero::throwing_environment::ThrowingEnvironment);
