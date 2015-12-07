// wsn pgm to receive Baxter trajectories and interpolate them smoothly
// as commands to Baxter;
// right arm only, at present; June 1, 2015
#include <baxter_traj_streamer/baxter_traj_streamer.h>

using namespace std;

bool bezier_curves(const trajectory_msgs::JointTrajectory trajectory, vector<Vectorq7x1> &Ct, vector<Vectorq7x1> &Cp);
void two_points_interpolate(vector<double> t,vector<double> positions,int n_control_points,vector<Vectorq7x1> &C_t, vector<Vectorq7x1> &C_p);
void bezier_interpolation(const vector<Vectorq7x1> C_t, const vector<Vectorq7x1> C_p, vector<Vectorq7x1> & qvecs, const double inter_t);
void find_control_points1(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, double delta, int j);
void find_control_points2(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, double delta, int j);
void find_control_points3(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope1, double slope2, double delta, int j);
void find_control_points4(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double delta, int j);
const int n_seg=200;
#define CALIBRATION

trajectory_msgs::JointTrajectory new_trajectory; // global var to receive new traj's;
bool got_new_trajectory = false;
bool working_on_trajectory = false;
baxter_core_msgs::JointCommand right_cmd,left_cmd;
ros::Publisher joint_cmd_pub_right;
// dt_traj is defined in the header file <baxter_traj_streamer/baxter_traj_streamer.h>
//double dt_traj = 0.01; // time step 
//complement of this:
// right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1);  
void trajectoryCb(const trajectory_msgs::JointTrajectory& tj_msg) {
    // copy trajectory to global var:
    new_trajectory = tj_msg; // does this work?
    // insist that a traj have at least 2 pts
    if (new_trajectory.points.size()>1) got_new_trajectory = true;
    cout<<"Cb received traj w/ npts = "<<new_trajectory.points.size()<<endl;
        trajectory_msgs::JointTrajectoryPoint trajectory_point0;
        trajectory_point0 = new_trajectory.points[0];  
        trajectory_point0 =  tj_msg.points[0];   
        cout<<tj_msg.points[0].positions.size()<<" =  tj_msg.points[0].positions.size()"<<endl;
        cout<<"size of positions[]: "<<trajectory_point0.positions.size()<<endl;
        cout<<"1st pt: ";
          for (int i=0;i<7;i++) { //copy from traj point to 7x1 vector
            cout<<trajectory_point0.positions[i]<<", ";           
          }
          cout<<endl;

} 

void cmd_pose_right(Vectorq7x1 qvec ) {
    //member var right_cmd_ already has joint names populated
    for (int i=0;i<7;i++) {
        right_cmd.command[i]=qvec[i];
    }
    joint_cmd_pub_right.publish(right_cmd);
}

bool trajInterpStatusSvc(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    //ROS_INFO("responding to service request: status of trajectory interpolator");
    response.resp = working_on_trajectory; // return status of "working on trajectory"
    return true;
}

// this is the interesting func: compute new qvec
// update isegment and qvec according to traj_clock; 
//if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false   
bool  update_trajectory(double traj_clock,trajectory_msgs::JointTrajectory trajectory,Vectorq7x1 qvec_prev, int &isegment,Vectorq7x1 &qvec_new){
    trajectory_msgs::JointTrajectoryPoint trajectory_point_from,trajectory_point_to;    
    Vectorq7x1 qvec,qvec_to,delta_qvec,dqvec;
    int nsegs = trajectory.points.size()-1;
    double t_subgoal;
    //cout<<"traj_clock = "<<traj_clock<<endl;
    if (isegment<nsegs) {
        trajectory_point_to = trajectory.points[isegment+1];
        t_subgoal = trajectory_point_to.time_from_start.toSec(); 
        //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    }
    else {
        cout<<"reached end of last segment"<<endl;
         trajectory_point_to = trajectory.points[nsegs];
         t_subgoal = trajectory_point_to.time_from_start.toSec();  
            for (int i=0;i<7;i++) {
                 qvec_new[i] = trajectory_point_to.positions[i]; 
            }   
         cout<<"final time: "<<t_subgoal<<endl;
            return false;        
    }
    
    //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    while ((t_subgoal< traj_clock)&&(isegment<nsegs)) { // The previous traj segment is done
        //cout<<"loop: iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
        isegment++;
        if (isegment>nsegs-1) {
            //last point
            trajectory_point_to = trajectory.points[nsegs];
            for (int i=0;i<7;i++) {
                 qvec_new[i] = trajectory_point_to.positions[i]; 
            }        
            cout<<"iseg>nsegs"<<endl;
            return false;
        }

       trajectory_point_to = trajectory.points[isegment+1];
       t_subgoal = trajectory_point_to.time_from_start.toSec();
    }
    //cout<<"t_subgoal = "<<t_subgoal<<endl;
    //here if have a valid segment:
    for (int i=0;i<7;i++) {
           qvec_to[i] = trajectory_point_to.positions[i]; 
    }
     delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
    double delta_time = t_subgoal-traj_clock;
    if (delta_time<dt_traj) delta_time= dt_traj;
    dqvec = delta_qvec*dt_traj/delta_time;
    qvec_new = qvec_prev + dqvec;
    return true; 
}

int main(int argc, char** argv){

  double traj_clock, dt_segment,dq_segment,delta_q_segment,traj_final_time;
  int isegment;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;

  Vectorq7x1 qvec,qvec0,qvec_prev,qvec_new;
    ros::init(argc, argv, "interpolator");
    ros::NodeHandle nh;
    
    ros::Subscriber traj_sub = nh.subscribe("right_arm_joint_path_command", 1, trajectoryCb); 
    //publisher is global
    joint_cmd_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1); 
    
    ROS_INFO("Initializing Services");
    ros::ServiceServer statusService = nh.advertiseService("trajInterpStatusSvc",trajInterpStatusSvc); 
    
    //initializations:
    left_cmd.mode = 1; // set the command modes to "position"
    right_cmd.mode = 1;

// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
  right_cmd.names.push_back("right_s0");
  right_cmd.names.push_back("right_s1");
  right_cmd.names.push_back("right_e0");
  right_cmd.names.push_back("right_e1");
  right_cmd.names.push_back("right_w0");
  right_cmd.names.push_back("right_w1");
  right_cmd.names.push_back("right_w2");
// same order for left arm
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");
  // do push-backs to establish desired vector size with valid joint angles
  for (int i=0;i<7;i++) {
     right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   } 
  
  ROS_INFO("ready to receive/execute trajectories");
  //main loop:
  vector<Vectorq7x1> Ct; // bezier curves: variable time
  vector<Vectorq7x1> Cp; // bezier curves: variable position
  vector<Vectorq7x1> qvecs; // bezier interpolation points
  int n_b; // number of bezier interpolation points
  int i_b=0; // index of bezier interpolation points
    while (ros::ok()) {
      if ((!working_on_trajectory)&&got_new_trajectory) {
          working_on_trajectory=true;
          got_new_trajectory=false;
          traj_clock=0.0; // initialize clock for trajectory;
          isegment=0;
          trajectory_point0 = new_trajectory.points[0];         
          for (int i=0;i<7;i++) { //copy from traj point to 7x1 vector
            qvec0[i] = trajectory_point0.positions[i];           
          }
          cmd_pose_right(qvec0);  //populate and send out first command  
          qvec_prev = qvec0;
          cout<<"start pt: "<< qvec0.transpose()<<endl;

          ROS_INFO("Getting Bezier Curves.");
          bezier_curves(new_trajectory, Ct, Cp);
          ROS_INFO("Got Bezier Curves.");
          ROS_INFO("Interpolating Bezier Curves.");
          bezier_interpolation(Ct,Cp, qvecs, dt_traj);
          ROS_INFO("Interpolation done.");
          ROS_INFO("sending command to robot.");
          n_b=qvecs.size();
          i_b=0;
          cout<<"the number of bezier Interpolation points:"<<n_b<<endl;
      }
      //working_on_trajectory = false;

/*      vector<double> traj_p,traj_t; // for find_control_pointsx debugging
      traj_t.clear();
      traj_t.push_back(0);
      traj_t.push_back(1);
      traj_t.push_back(2);
      traj_p.clear();
      traj_p.push_back(0);
      traj_p.push_back(1);
      traj_p.push_back(0);
      vector<double> C_p, C_t;
      find_control_points3(traj_p,traj_t,C_p,C_t,0.2,1,0.5,0);*/   

/*          vector<Vectorq7x1> Ct; // for bezier_interpolation debugging
          vector<Vectorq7x1> Cp;
          vector<Vectorq7x1> qvecs;
          Vectorq7x1 temp;
          temp<<0   ,0   ,0   ,0   ,0   ,0   ,0   ;
          Ct.push_back(temp);
          temp<<0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ;
          Ct.push_back(temp);
          temp<<0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ;
          Ct.push_back(temp);
          temp<<0.21 ,0.21 ,0.21 ,0.21 ,0.21 ,0.21 ,0.21 ;
          Ct.push_back(temp);
          temp<<0   ,0   ,0   ,0   ,0   ,0   ,0   ;
          Cp.push_back(temp);
          temp<<0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ;
          Cp.push_back(temp);
          temp<<0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ;
          Cp.push_back(temp);
          temp<<-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1;
          Cp.push_back(temp);
          bezier_interpolation(Ct,Cp, qvecs, dt_traj);
          working_on_trajectory=false;*/


      if (working_on_trajectory) {
/*          traj_clock+=dt_traj;
          // update isegment and qvec according to traj_clock; 
          //if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false          
          working_on_trajectory = update_trajectory(traj_clock,new_trajectory,qvec_prev,isegment,qvec_new);
          cmd_pose_right(qvec_new); // use qvec to populate object and send it to robot
          qvec_prev = qvec_new;
          cout<<"traj_clock: "<<traj_clock<<"; vec:"<<qvec_new.transpose()<<endl;
          if (!working_on_trajectory)
              cout<<"completed execution of a trajectory"<<endl;
*/
          cmd_pose_right(qvecs[i_b]);
          i_b++;
          if(i_b==n_b-1) 
          {
            working_on_trajectory=false; // end condition; i is from 0 to n-1
            ROS_INFO("completed execution of a trajectory");
          }
      }
    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }    
}
  
void bezier_interpolation(const vector<Vectorq7x1> C_t, const vector<Vectorq7x1> C_p, vector<Vectorq7x1> & qvecs, const double inter_t)
{
    int n_points=C_p.size(); // all Bezier points
    double sum_t=C_t[n_points-1][0]-C_t[0][0]; // all joints have common time stamp;
    int n_inter = sum_t/inter_t+1; // the number of interpolation points including the start point
    double t_clock=0; // time stamp
    int t_prev=0; // time index at previous point
    int t_next=0; // time index at next point
    Vectorq7x1 slope; 
    qvecs.resize(n_inter);  
    qvecs[0]=C_p[0]; // start point
    for (int i = 1; i < n_inter; i++) // from the second point
    {
        t_clock += inter_t;
        while(C_t[t_next][0]<t_clock)
        {
            t_next++;
        }
        t_prev = t_next-1;
        slope=(C_p[t_next]-C_p[t_prev])/(C_t[t_next][0]-C_t[t_prev][0]);
        qvecs[i]=C_p[t_prev]+slope*(t_clock-C_t[t_prev][0]);
    }

    #ifdef CALIBRATION
      if(qvecs[n_inter-1]!=C_p[n_points-1])
        qvecs.push_back(C_p[n_points-1]);
    #endif

    cout<<"-----------------------------"<<endl;
    n_inter=qvecs.size();
    for(int i = 0; i < n_inter; i++)
    {
        cout<<qvecs[i].transpose()<<endl;
    }
}

bool bezier_curves(const trajectory_msgs::JointTrajectory trajectory, vector<Vectorq7x1> &Ct, vector<Vectorq7x1> &Cp)
{
  	int n_points = trajectory.points.size(); // the number of trajectory points
  	ROS_INFO("There are %d points to be interpolate including the start pose.", n_points);
  	int n_control_points = (n_points-1)*4*7; // the number of de Boor control points
  	// there are n_points-1 Bezier curves. For each curve there are 4 control points. And there are 7 joints.
//  	vector<Eigen::Vector2d> Control_Points; // de Boor control points. (time,position)
  	vector<double> t; // time for control point
  	vector<double> positions; // position for control point
  	double delta=0.4; // tangent coeficient
//  	Control_Points.resize(n_control_points);
  	t.resize(n_control_points);
  	positions.resize(n_control_points);
  	Ct.clear();
 	  Cp.clear();
 	  Ct.resize((n_seg+1)*(n_points-1));
 	  Cp.resize((n_seg+1)*(n_points-1));
  	if (n_points == 2) // 2 points
  	{
  		double start_t = trajectory.points[0].time_from_start.toSec();
  		double end_t = trajectory.points[1].time_from_start.toSec();
  		double delta_t = end_t - start_t;
  		for(int i = 0; i < n_control_points; i++)
  		{
  			if(i%4==0)  
  			{
  				t[i] = start_t;
  				positions[i]=trajectory.points[0].positions[i/4];
  			}
  			if(i%4==3)  
  			{
  				t[i] = end_t;
  				positions[i]=trajectory.points[1].positions[i/4];
  			}
  			if(i%4==1)  
  			{
  				t[i] = start_t+delta*delta_t;
  				positions[i]=trajectory.points[0].positions[i/4];
  			}
  			if(i%4==2)  
  			{
  				t[i] = end_t-delta*delta_t;
  				positions[i]=trajectory.points[1].positions[i/4];
  			}
  			//cout<<i<<" t:"<<t[i]<<endl<<"postion:"<<positions[i]<<endl;
  		}// loop ends
  		two_points_interpolate(t,positions,n_control_points,Ct,Cp);
  		return true;
  	} // 2 points

 	if(n_points>2)
 	{
 		vector<double> traj_p;
 		vector<double> traj_t;
 		vector<double> C_t;
 		vector<double> C_p;
 		
 		for(int i = 0; i < 7; i++ )
 		{
 			traj_p.clear();
 			traj_p.resize(n_points);
			traj_t.clear();
 			traj_t.resize(n_points);
 			C_t.clear();
 			C_p.clear();
 			for(int j = 0; j < n_points; j++)
 			{
 				traj_p[j]=trajectory.points[j].positions[i];
 				traj_t[j]=trajectory.points[j].time_from_start.toSec();
 			}

 			for(int j = 0; j < n_points-2; j++)
 			{
 				if((traj_p[j+1]-traj_p[j])*(traj_p[j+2]-traj_p[j+1])>0) // need to calculate slope at next point
 				{
 					
 					if(j==0)
 					{
 						double slope;
 						double k1,k2;
 						vector<double> t;
 						vector<double> p;
 						t.clear();
 						p.clear();
 					  	k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
 					  	k2=(traj_p[j+2]-traj_p[j+1])/(traj_t[j+2]-traj_t[j+1]);
 					  	slope=tan((atan(k1)+atan(k2))/2);
 						find_control_points1(traj_p,traj_t,p,t,slope,delta,j);
 						int n=p.size();
 						for(int k=0; k < n; k++)
 						{
 							C_t.push_back(t[k]);
 							C_p.push_back(p[k]);
 						}

 						if(j==n_points-3) // there are only three points
 						{

	 						find_control_points2(traj_p,traj_t,p,t,slope,delta,j+1);
	 						int n=p.size();
	 						for(int k=0; k < n; k++)
	 						{
	 							C_t.push_back(t[k]);
	 							C_p.push_back(p[k]);
	 						}
 						}

 					}

 					if( j > 0 )
 					{
 						if((traj_p[j+1]-traj_p[j])*(traj_p[j]-traj_p[j-1])>0) // need to calculate the slope at this point
 						{
 							double slope1,slope2;
 							double k1,k2;
 						 	vector<double> t;
 							vector<double> p;
 							t.clear();
 							p.clear();
 					  		k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
 					  		k2=(traj_p[j+2]-traj_p[j+1])/(traj_t[j+2]-traj_t[j+1]);
 					  		slope1=tan((atan(k1)+atan(k2))/2);
							
							k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
 					  		k2=(traj_p[j]-traj_p[j-1])/(traj_t[j]-traj_t[j-1]);
							slope2=tan((atan(k1)+atan(k2))/2);

 							find_control_points3(traj_p,traj_t,p,t,slope1,slope2,delta,j);
 							int n=p.size();
 							for(int k=0; k < n; k++)
 							{
 								C_t.push_back(t[k]);
 								C_p.push_back(p[k]);
 							}	
 						}
 						else  // the slope is zero at this point
 						{
 							double slope;
 							double k1,k2;
 							vector<double> t;
 							vector<double> p;
 							t.clear();
 								p.clear();
 						  	k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
 						  	k2=(traj_p[j]-traj_p[j-1])/(traj_t[j]-traj_t[j-1]);
 						  	slope=tan((atan(k1)+atan(k2))/2);
 							find_control_points1(traj_p,traj_t,p,t,slope,delta,j);
 							int n=p.size();
 							for(int k=0; k < n; k++)
 							{
 								C_t.push_back(t[k]);
 								C_p.push_back(p[k]);
 							}
 						}


 						if(j==n_points-3) // there are two points left
 						{
 							double slope;
	 						double k1,k2;
	 						vector<double> t;
	 						vector<double> p;
	 						t.clear();
	 						p.clear();
	 					  	k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
	 					  	k2=(traj_p[j+2]-traj_p[j+1])/(traj_t[j+2]-traj_t[j+1]);
	 					  	slope=tan((atan(k1)+atan(k2))/2);
	 						find_control_points2(traj_p,traj_t,p,t,slope,delta,j+1);
	 						int n=p.size();
	 						for(int k=0; k < n; k++)
	 						{
	 							C_t.push_back(t[k]);
	 							C_p.push_back(p[k]);
	 						}
 						}

 					} // need to calculate the slope at next point

 				}
 				else // slope is zero at next point
 				{
  					if(j==0)
 					{
 						vector<double> t;
 						vector<double> p;
 						t.clear();
 						p.clear();
 						find_control_points4(traj_p,traj_t,p,t,delta,j);
 						int n=p.size();
 						for(int k=0; k < n; k++)
 						{
 							C_t.push_back(t[k]);
 							C_p.push_back(p[k]);
 						}

 						if(j==n_points-3) // there are only three points
 						{
 						 	vector<double> t;
 							vector<double> p;
 							t.clear();
 							p.clear();
	 						find_control_points4(traj_p,traj_t,p,t,delta,j+1);
	 						int n=p.size();
	 						for(int k=0; k < n; k++)
	 						{
	 							C_t.push_back(t[k]);
	 							C_p.push_back(p[k]);
	 						}
 						} 						
 					} // j==0;
 					if( j > 0 )
 					{
 						if((traj_p[j+1]-traj_p[j])*(traj_p[j]-traj_p[j-1])>0) // need to calculate the slope at this point
 						{
 							double slope;
 							double k1,k2;
 						 	vector<double> t;
 							vector<double> p;
 							t.clear();
 							p.clear();
							k1=(traj_p[j+1]-traj_p[j])/(traj_t[j+1]-traj_t[j]);
 					  		k2=(traj_p[j]-traj_p[j-1])/(traj_t[j]-traj_t[j-1]);
							slope=tan((atan(k1)+atan(k2))/2);

 							find_control_points2(traj_p,traj_t,p,t,slope,delta,j);
 							int n=p.size();
 							for(int k=0; k < n; k++)
 							{
 								C_t.push_back(t[k]);
 								C_p.push_back(p[k]);
 							}	
 						}
 						else // the slope is zero at this point
 						{
 						 	vector<double> t;
 							vector<double> p;
 							t.clear();
 							p.clear();
 							find_control_points4(traj_p,traj_t,p,t,delta,j);
 							int n=p.size();
 							for(int k=0; k < n; k++)
 							{
 								C_t.push_back(t[k]);
 								C_p.push_back(p[k]);
 							}
 						}

 						if(j==n_points-3) // there are two points left
 						{
 						 	vector<double> t;
 							vector<double> p;
 							t.clear();
 							p.clear();
	 						find_control_points4(traj_p,traj_t,p,t,delta,j+1);
	 						int n=p.size();
	 						for(int k=0; k < n; k++)
	 						{
	 							C_t.push_back(t[k]);
	 							C_p.push_back(p[k]);
	 						}
 						}

 					} // j>0					
 				}// slope is zero at next point

 			}// set all points j
 			int n=C_p.size();
 			cout<<"joint"<<i+1<<endl;
 			for(int k = 0; k < n; k++)
 			{
 				Cp[k][i]=C_p[k];
 				Ct[k][i]=C_t[k];
 				cout<<"t:"<<Ct[k][i]<<"; p:"<<Cp[k][i]<<endl;
 			}

 		}// set all joints i

 		int n=Ct.size();
 		for(int i = 0; i < n; i++)
 		{
 			cout<<"seg:"<<i<<endl;
 			for(int j = 0; j < 7; j++)
 			{
 				cout<<"joint"<<j+1<<", t:"<<Ct[i][j]<<"; p:"<<Cp[i][j]<<endl;
 			}
 		}

 	}// n_points>2 ends 
  	return true;
}

void find_control_points1(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, double delta, int j)
{
	vector<double> t;
 	vector<double> p;
 	t.resize(4);
 	p.resize(4);
	t[0]=traj_t[j];
	p[0]=traj_p[j];

	t[3]=traj_t[j+1];
	p[3]=traj_p[j+1];

	double delta_t = t[3]-t[0];
	t[1]=t[0]+delta_t*delta;
	p[1]=p[0];

	t[2]=t[3]-delta_t*delta;
	p[2]=p[3]-delta_t*delta*slope;


  	double b_t=0; // bezier time
  	C_t.clear();
  	C_p.clear();
  	double C1,C2;
  	for(int i = 0; i < n_seg+1; i++)
  	{
  		cout<<"seg:"<<i<<endl;
  		C1=(1-b_t)*(1-b_t)*(1-b_t)*t[0]+
  		3*(1-b_t)*(1-b_t)*b_t*t[1]+
  		3*(1-b_t)*b_t*b_t*t[2]+
  		b_t*b_t*b_t*t[3]; // weird: operator ^ cannot be used.

  			
  		C2=(1-b_t)*(1-b_t)*(1-b_t)*p[0]+
  		3*(1-b_t)*(1-b_t)*b_t*p[1]+
  		3*(1-b_t)*b_t*b_t*p[2]+
		b_t*b_t*b_t*p[3];
  		cout<<"joint:"<<" t:"<<C1<<"; p:"<<C2<<endl;
  		b_t=b_t+1.0/n_seg;
  		C_t.push_back(C1);
  		C_p.push_back(C2);
  	}
}

void find_control_points2(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, double delta, int j)
{
	vector<double> t;
 	vector<double> p;
 	t.resize(4);
 	p.resize(4);
	t[0]=traj_t[j];
	p[0]=traj_p[j];

	t[3]=traj_t[j+1];
	p[3]=traj_p[j+1];

	double delta_t = t[3]-t[0];
	t[1]=t[0]+delta_t*delta;
	p[1]=p[0]+delta_t*delta*slope;

	t[2]=t[3]-delta_t*delta;
	p[2]=p[3];


  	double b_t=0; // bezier time
  	C_t.clear();
  	C_p.clear();
  	double C1,C2;
  	for(int i = 0; i < n_seg+1; i++)
  	{
  		cout<<"seg:"<<i<<endl;
  		C1=(1-b_t)*(1-b_t)*(1-b_t)*t[0]+
  		3*(1-b_t)*(1-b_t)*b_t*t[1]+
  		3*(1-b_t)*b_t*b_t*t[2]+
  		b_t*b_t*b_t*t[3]; // weird: operator ^ cannot be used.

  			
  		C2=(1-b_t)*(1-b_t)*(1-b_t)*p[0]+
  		3*(1-b_t)*(1-b_t)*b_t*p[1]+
  		3*(1-b_t)*b_t*b_t*p[2]+
		b_t*b_t*b_t*p[3];
  		cout<<"joint:"<<" t:"<<C1<<"; p:"<<C2<<endl;
  		b_t=b_t+1.0/n_seg;
  		C_t.push_back(C1);
  		C_p.push_back(C2);
  	}

}
void find_control_points3(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope1, double slope2, double delta, int j)
{
	vector<double> t;
 	vector<double> p;
 	t.resize(4);
 	p.resize(4);
	t[0]=traj_t[j];
	p[0]=traj_p[j];

	t[3]=traj_t[j+1];
	p[3]=traj_p[j+1];

	double delta_t = t[3]-t[0];
	t[1]=t[0]+delta_t*delta;
	p[1]=p[0]+delta_t*delta*slope1;

	t[2]=t[3]-delta_t*delta;
	p[2]=p[3]-delta_t*delta*slope2;


  	double b_t=0; // bezier time
  	C_t.clear();
  	C_p.clear();
  	double C1,C2;
  	for(int i = 0; i < n_seg+1; i++)
  	{
  		cout<<"seg:"<<i<<endl;
  		C1=(1-b_t)*(1-b_t)*(1-b_t)*t[0]+
  		3*(1-b_t)*(1-b_t)*b_t*t[1]+
  		3*(1-b_t)*b_t*b_t*t[2]+
  		b_t*b_t*b_t*t[3]; // weird: operator ^ cannot be used.

  			
  		C2=(1-b_t)*(1-b_t)*(1-b_t)*p[0]+
  		3*(1-b_t)*(1-b_t)*b_t*p[1]+
  		3*(1-b_t)*b_t*b_t*p[2]+
		b_t*b_t*b_t*p[3];
  		cout<<"joint:"<<" t:"<<C1<<"; p:"<<C2<<endl;
  		b_t=b_t+1.0/n_seg;
  		C_t.push_back(C1);
  		C_p.push_back(C2);
  	}
}

void find_control_points4(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double delta, int j)
{
	vector<double> t;
 	vector<double> p;
 	t.resize(4);
 	p.resize(4);
	t[0]=traj_t[j];
	p[0]=traj_p[j];

	t[3]=traj_t[j+1];
	p[3]=traj_p[j+1];

	double delta_t = t[3]-t[0];
	t[1]=t[0]+delta_t*delta;
	p[1]=p[0];

	t[2]=t[3]-delta_t*delta;
	p[2]=p[3];


  	double b_t=0; // bezier time
  	C_t.clear();
  	C_p.clear();
  	double C1,C2;
  	for(int i = 0; i < n_seg+1; i++)
  	{
  		cout<<"seg:"<<i<<endl;
  		C1=(1-b_t)*(1-b_t)*(1-b_t)*t[0]+
  		3*(1-b_t)*(1-b_t)*b_t*t[1]+
  		3*(1-b_t)*b_t*b_t*t[2]+
  		b_t*b_t*b_t*t[3]; // weird: operator ^ cannot be used.

  			
  		C2=(1-b_t)*(1-b_t)*(1-b_t)*p[0]+
  		3*(1-b_t)*(1-b_t)*b_t*p[1]+
  		3*(1-b_t)*b_t*b_t*p[2]+
		b_t*b_t*b_t*p[3];
  		cout<<"joint:"<<" t:"<<C1<<"; p:"<<C2<<endl;
  		b_t=b_t+1.0/n_seg;
  		C_t.push_back(C1);
  		C_p.push_back(C2);
  	}
}

void two_points_interpolate(vector<double> t,vector<double> positions,int n_control_points,vector<Vectorq7x1> &C_t, vector<Vectorq7x1> &C_p)
{
  	double b_t=0; // bezier time
  	C_t.clear();
  	C_p.clear();
  	for(int i = 0; i < n_seg+1; i++)
  	{
  		Vectorq7x1 C1,C2;
  		cout<<"seg:"<<i<<endl;
  		for(int j = 0; j < n_control_points-1; )
  		{
  			C1[j/4]=(1-b_t)*(1-b_t)*(1-b_t)*t[j]+
  			3*(1-b_t)*(1-b_t)*b_t*t[j+1]+
  			3*(1-b_t)*b_t*b_t*t[j+2]+
  			b_t*b_t*b_t*t[j+3]; // weird: operator ^ cannot be used.

  			
  			C2[j/4]=(1-b_t)*(1-b_t)*(1-b_t)*positions[j]+
  			3*(1-b_t)*(1-b_t)*b_t*positions[j+1]+
  			3*(1-b_t)*b_t*b_t*positions[j+2]+
  			b_t*b_t*b_t*positions[j+3];
  			cout<<"joint:"<<j/4+1<<" t:"<<C1[j/4]<<"; p:"<<C2[j/4]<<endl;
  			j+=4;
  		}
  		b_t=b_t+1.0/n_seg;
  		C_t.push_back(C1);
  		C_p.push_back(C2);
  	}
}
