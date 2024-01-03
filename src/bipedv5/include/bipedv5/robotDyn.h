
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <string>
#include <vector>
#include <map>

#include <iostream>
#include <Eigen/Dense>


using namespace std;

class robotDyn { 
    private:
        pinocchio::Model *pmodel_;
        pinocchio::Data *data_;
        std::vector<std::string> jnames_;
        Eigen::VectorXd qmin_, qmax_, dqmax_, q_, qdot_, qd, qdotd;
        bool has_floating_base;
        Eigen::Vector3d vwb, omegawb, pwb;
        Eigen::Matrix3d Rwb;
        Eigen::Affine3d Twb;
        Eigen::Quaterniond qwb;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        robotDyn(const std::string &model_name, const bool &has_floating_base_){
            has_floating_base = has_floating_base_;
            pmodel_ = new pinocchio::Model();
            if (has_floating_base)  //true
                pinocchio::urdf::buildModel(model_name, pinocchio::JointModelFreeFlyer(),
                                            *pmodel_, false);
            else
                pinocchio::urdf::buildModel(model_name, *pmodel_, false);
            data_ = new pinocchio::Data(*pmodel_);
            jnames_.clear();
            int names_size = pmodel_->names.size();
            jnames_.reserve(names_size);
            //jnames去除了 universe和root_joint,ndof维度
            for (int i = 0; i < names_size; i++)
            {
                const std::string &jname = pmodel_->names[i];
                std::cout << "Joint Id: " << pmodel_->getJointId(jname) << " # " << pmodel_->idx_qs[pmodel_->getJointId(jname)] << " " << pmodel_->idx_vs[pmodel_->getJointId(jname)] << " Name: " << jname << std::endl;
                if (jname.compare("universe") != 0 && jname.compare("root_joint") != 0)
                {
                    jnames_.push_back(jname);
                }
            }
            cout<<"name size:"<<names_size<<" jnmae size: "<<jnames_.size()<<endl;
            qmin_.resize(pmodel_->nq);      //ndof+7
            qmax_.resize(pmodel_->nq);
            dqmax_.resize(pmodel_->nv);     //ndof+6
            qmin_ = pmodel_->lowerPositionLimit;
            qmax_ = pmodel_->upperPositionLimit;
            dqmax_ = pmodel_->velocityLimit;        //ndof+6,包含了浮动基，浮动基速度上限为无限大

            qd.setZero(pmodel_->nq);        //控制指令
            q_.setZero(pmodel_->nq);           //真实转角
            qdotd.setZero(pmodel_->nv);
            qdot_.setZero(pmodel_->nv);
            std::cout << "Model loaded: " << model_name << std::endl;
        }
        
        void setBaseToWorldState(Eigen::Vector3d pwb_, Eigen::Quaterniond qwb_)
        {
            qwb = qwb_;
            pwb = pwb_;
            Rwb = qwb_.toRotationMatrix();
            Twb.translation() = pwb;
            Twb.linear() = Rwb;
        }

        void setBaseWorldVelocity(Eigen::Vector3d vwb_, Eigen::Vector3d omegawb_)
        {
            vwb = vwb_;
            omegawb = omegawb_;
        }


        void mapJointNamesIDs(const std::vector<std::string> &jnames,      //ndof维
                                        const std::vector<double> &qvec,
                                        const std::vector<double> &qdotvec)
        {
        // assert(qvec.size() == jnames.size() && qdotvec.size() == jnames.size());  

            for (int i = 0; i < jnames.size(); i++)
            {
                int jidx = pmodel_->getJointId(jnames[i]);      //关节编号
                int qidx = pmodel_->idx_qs[jidx];               //角度编号
                int vidx = pmodel_->idx_vs[jidx];               //速度编号
            //   cout<<"i: "<<i<<",name: "<<jnames[i]<<",jidx: "<<jidx<<", qidx: "<<qidx<<", vidx: "<<vidx<<endl;
                //this value is equal to 2 for continuous joints
                if (pmodel_->nqs[jidx] == 2)
                {
                    q_[qidx] = cos(qvec[i]);
                    q_[qidx + 1] = sin(qvec[i]);
                    qdot_[vidx] = qdotvec[i];
                }
                else
                {
                    q_[qidx] = qvec[i];
                    qdot_[vidx] = qdotvec[i];
                }
            }
        }


        void updateJointConfig(const std::vector<std::string> &jnames,
                                            const std::vector<double> &qvec,
                                            const std::vector<double> &qdotvec)
        {

            q_.setZero(pmodel_->nq);    //ndof+7
            qdot_.setZero(pmodel_->nv); //ndof+6
            mapJointNamesIDs(jnames, qvec, qdotvec);        //不含浮动基部分,q_(7:end),qodt_(6:end)

            if (has_floating_base)
            {
                //position
                q_[0] = pwb(0);
                q_[1] = pwb(1);
                q_[2] = pwb(2);
                q_[3] = qwb.x(); //x
                q_[4] = qwb.y(); //y
                q_[5] = qwb.z(); //z
                q_[6] = qwb.w(); //w

                //Velocity
                qdot_[0] = vwb(0);
                qdot_[1] = vwb(1);
                qdot_[2] = vwb(2);
                qdot_[3] = omegawb(0); //x
                qdot_[4] = omegawb(1); //y
                qdot_[5] = omegawb(2); //z
            }
            //更新运动学
            pinocchio::forwardKinematics(*pmodel_,*data_,q_,qdot_);
            pinocchio::updateFramePlacements(*pmodel_, *data_);
            pinocchio::computeJointJacobians(*pmodel_, *data_, q_);

        }

        Eigen::VectorXd comPosition()
        {
            pinocchio::centerOfMass(*pmodel_, *data_, q_);
            // std::cout<<"actual: "<<q_.transpose()<<std::endl;
            return data_->com[0];
        }

};
