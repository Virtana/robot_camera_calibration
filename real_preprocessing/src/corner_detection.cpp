#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "sstream"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Dense"
#include "fstream"
#include "string"

void tfCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
    Eigen::MatrixXd intrinsic(3,3); //potential calibration function
    intrinsic << 630.3467672874693, 0, 314.10832937485145,
         0, 624.9044125522948, 241.59156511756711,
         0, 0, 1;  
    if (msg->detections.empty()){
        //no detections
    }
    else{
        std::string filename("detections_0000.yml");//output path
        std::ofstream fout;
        fout.open(filename.c_str());
        fout<<"detections:";

        for (float i=(msg->detections.size()-1);i>-1;i--){
            //reading data from topic 
            float xpos=msg->detections[i].pose.pose.pose.position.x;
            float ypos=msg->detections[i].pose.pose.pose.position.y;
            float zpos=msg->detections[i].pose.pose.pose.position.z;
            float xori=msg->detections[i].pose.pose.pose.orientation.x;
            float yori=msg->detections[i].pose.pose.pose.orientation.y;
            float zori=msg->detections[i].pose.pose.pose.orientation.z;
            float wori=msg->detections[i].pose.pose.pose.orientation.w;
            float size=msg->detections[i].size[0];
            int id=msg->detections[i].id[0]; 

            tf::Quaternion q(xori,yori,zori,wori);
            tf::Matrix3x3 m(q); //quaternion to rotational matrix
      
            Eigen::MatrixXd trans(3,4);// 3x4 transformation matrix 
            trans << m[0][0],m[0][1],m[0][2],xpos,
                    m[1][0],m[1][1],m[1][2], ypos,
                    m[2][0],m[2][1],m[2][2], zpos;
           
            Eigen::MatrixXd cornertl(4,1); // 4x1 (X,Y,Z,1) for each tag corner
            cornertl << -(size/2),
                (size/2),
                0,
                1;
            Eigen::MatrixXd cornertr(4,1);
            cornertr << (size/2),
                (size/2),
                0,
                1;
            Eigen::MatrixXd cornerbl(4,1);
            cornerbl << -(size/2),
                -(size/2),
                0,
                1;
            Eigen::MatrixXd cornerbr(4,1);
            cornerbr << (size/2),
                -(size/2),
                0,
                1;       

            Eigen::MatrixXd pixtl(3,1); 
            pixtl=(intrinsic*trans)*cornertl; //pixel coordinate calculation
            pixtl << (pixtl(0,0)/pixtl(2,0)),  //coordinate scaling
                     (pixtl(1,0)/pixtl(2,0)),
                     1;
            std::string tl("[ "+std::to_string(int(pixtl(0,0)))+", "+std::to_string(int(pixtl(1,0)))+" ]");
            Eigen::MatrixXd pixtr(3,1);
            pixtr=(intrinsic*trans)*cornertr; 
            pixtr << (pixtr(0,0)/pixtr(2,0)),
                     (pixtr(1,0)/pixtr(2,0)),
                     1;
            std::string tr("[ "+std::to_string(int(pixtr(0,0)))+", "+std::to_string(int(pixtr(1,0)))+" ]");
            Eigen::MatrixXd pixbr(3,1);  
            pixbr=(intrinsic*trans)*cornerbr;
            pixbr << (pixbr(0,0)/pixbr(2,0)),
                     (pixbr(1,0)/pixbr(2,0)),
                     1;
            std::string br("[ "+std::to_string(int(pixbr(0,0)))+", "+std::to_string(int(pixbr(1,0)))+" ]"); 
               Eigen::MatrixXd pixbl(3,1);
            pixbl=(intrinsic*trans)*cornerbl;
            pixbl << (pixbl(0,0)/pixbl(2,0)),
                     (pixbl(1,0)/pixbl(2,0)),
                     1;
            std::string bl("[ "+std::to_string(int(pixbl(0,0)))+", "+std::to_string(int(pixbl(1,0)))+" ]"); 
                 
            fout << "\n - TargetID: "<<std::to_string(id); //yaml output formatting
            fout << "\n   corners:";                              
            fout << "\n    0: "<<tl;
            fout << "\n    1: "<<tr;
            fout << "\n    2: "<<br;
            fout << "\n    3: "<<bl;
        }
        fout.close();
    }
}

int main (int argc, char** argv){
    ros::init(argc,argv,"corner_detection");
    ros::NodeHandle nh;
    ros::Subscriber sub= nh.subscribe("tag_detections",10,tfCallback);
    ros::spin();
    return 0;
}


