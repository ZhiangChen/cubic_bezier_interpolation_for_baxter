#include <bezier_curves/bezier_interpolator.h>
using namespace std;


Bezier_interpolator::Bezier_interpolator(int seg, double inter_t, double delta):n_seg(seg),inter_t(inter_t),delta(delta),calibration(false){}

Bezier_interpolator::Bezier_interpolator(double inter_t):n_seg(200),inter_t(inter_t),delta(0.32),calibration(false){}

bool Bezier_interpolator::Bezier_interpolator_solver(const trajectory_msgs::JointTrajectory new_trajectory, vector<Vectorq7x1> & qvecs, int & n_b)
{
	vector<Vectorq7x1> Ct; // bezier curves: variable time
  	vector<Vectorq7x1> Cp; // bezier curves: variable position
  	int i_b=0; // index of bezier interpolation points
  	ROS_INFO("Getting Bezier Curves.");
    bezier_curves(new_trajectory, Ct, Cp);
    ROS_INFO("Got Bezier Curves.");
    ROS_INFO("Interpolating Bezier Curves.");
    bezier_interpolation(Ct,Cp, qvecs, dt_traj);
    ROS_INFO("Interpolation done.");
    n_b=qvecs.size();
    i_b=0;
    cout<<"The number of Bezier Interpolation points:"<<n_b<<endl;
    return true;
}

void Bezier_interpolator::bezier_interpolation(const vector<Vectorq7x1> C_t, const vector<Vectorq7x1> C_p, vector<Vectorq7x1> & qvecs, const double inter_t)
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

    if(calibration)
    {
      if(qvecs[n_inter-1]!=C_p[n_points-1])
      {
      	qvecs.push_back(C_p[n_points-1]);
      }
	}
    

    cout<<"---------------------------------------"<<endl;
    n_inter=qvecs.size();
    for(int i = 0; i < n_inter; i++)
    {
        cout<<qvecs[i].transpose()<<endl;
    }
}

bool Bezier_interpolator::bezier_curves(const trajectory_msgs::JointTrajectory trajectory, vector<Vectorq7x1> &Ct, vector<Vectorq7x1> &Cp)
{
  	int n_points = trajectory.points.size(); // the number of trajectory points
  	ROS_INFO("There are %d points to be interpolate including the start pose.", n_points);
  	int n_control_points = (n_points-1)*4*7; // the number of de Boor control points
  	// there are n_points-1 Bezier curves. For each curve there are 4 control points. And there are 7 joints.
//  	vector<Eigen::Vector2d> Control_Points; // de Boor control points. (time,position)
  	vector<double> t; // time for control point
  	vector<double> positions; // position for control point
//  	double delta=0.4; // tangent coeficient
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
 						find_control_points1(traj_p,traj_t,p,t,slope,j);
 						int n=p.size();
 						for(int k=0; k < n; k++)
 						{
 							C_t.push_back(t[k]);
 							C_p.push_back(p[k]);
 						}

 						if(j==n_points-3) // there are only three points
 						{

	 						find_control_points2(traj_p,traj_t,p,t,slope,j+1);
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

 							find_control_points3(traj_p,traj_t,p,t,slope1,slope2,j);
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
 							find_control_points1(traj_p,traj_t,p,t,slope,j);
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
	 						find_control_points2(traj_p,traj_t,p,t,slope,j+1);
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
 						find_control_points4(traj_p,traj_t,p,t,j);
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
	 						find_control_points4(traj_p,traj_t,p,t,j+1);
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

 							find_control_points2(traj_p,traj_t,p,t,slope,j);
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
 							find_control_points4(traj_p,traj_t,p,t,j);
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
	 						find_control_points4(traj_p,traj_t,p,t,j+1);
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

void Bezier_interpolator::find_control_points1(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, int j)
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

void Bezier_interpolator::find_control_points2(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, int j)
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
void Bezier_interpolator::find_control_points3(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope1, double slope2, int j)
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

void Bezier_interpolator::find_control_points4(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, int j)
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

void Bezier_interpolator::two_points_interpolate(vector<double> t,vector<double> positions,int n_control_points,vector<Vectorq7x1> &C_t, vector<Vectorq7x1> &C_p)
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