#ifndef __THROWING_ENVIRONMENT_H__
#define __THROWING_ENVIRONMENT_H__

#include <string>
#include <vector>
#include <mars/utils/Vector.h>
#include <Eigen/Dense>
#include <MARSEnvironment.h>
#include <ContextualEnvironment.h>

namespace bolero {
  namespace throwing_environment {

    class ThrowingEnvironment : public mars_environment::MARSEnvironment,
                                public ContextualEnvironment {

    public:
      ThrowingEnvironment(lib_manager::LibManager *theManager);
      ~ThrowingEnvironment();

      int getLibVersion() const {return 1;}
      const std::string getLibName() const {
        return std::string("throwing_environment");
      }

      virtual void initMARSEnvironment();
      virtual void resetMARSEnvironment();
      virtual void handleMARSError();

      virtual int getNumInputs() const;
      virtual int getNumOutputs() const;

      virtual void createOutputValues();
      virtual void handleInputValues();

      virtual int getFeedback(double *feedback) const;

      bool isEvaluationDone() const;
      bool isBehaviorLearningDone() const { return false; }

      virtual double* request_context(double *context,int size);
      virtual int get_num_context_dims() const;

    private:
      Eigen::VectorXd startAnglePos;
      int verbose;
      const double MAX_SIMULATION_TIME;
      double ballThrowTime;
      const unsigned int numJoints;
      double armHeight;
      bool isSceneLoaded;

      double fitness;
      double ballHitX;
      double ballHitY;
      bool evaluation_done;

      std::vector<unsigned long> motorIDs;

      mars::utils::Vector targetPos;

      void readConfig();
      std::string getConfigPath();
      void getMotorIDs();

      void setStartAngles();

      void outputMotorPositions();
      void setPositionOfVisualTarget();
      void checkBallPosition();
      void checkMaxTime();

      void setMotorValues();
      void checkBallThrowTime();

    };
  }
}

#endif
