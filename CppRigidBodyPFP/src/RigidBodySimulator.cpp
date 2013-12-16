#include "RigidBodySimulator.hpp"

RigidBodySimulator::RigidBodySimulator(void)
{
	//change to initialize every robot at a different location, positioned across the workspace at a uniform-random distribution
	/*NOTE: number of robots not known til ReadRobot is called,
	must create a way to find it by time of or on constructor call*/
	for(int i=0; i</*NUMBER OF ROBOTS*/; i++)
	{
		//m_robot.m_x = m_robot.m_y = m_robot.m_theta = 0;    
		//Must know the dimensions of workspace to use as range when generating coordinates
		m_robot.m_x[i] = ;/*rand using x-range coordinates*/
		m_robot.m_y[i] = ;/*rand using y-range coordinates*/
		m_robot.m_theta[i] = 0;
	}
	m_circles.push_back(16);
	m_circles.push_back(-7);
	m_circles.push_back(1.0);
}

RigidBodySimulator::~RigidBodySimulator(void)
{
}

//no change for this method as the x,y coordinates are passed in
	//I think...
	//need to investigate where it's being called
Point RigidBodySimulator::ClosestPointOnObstacle(const int i, const double x, const double y)
{
    const double cx = m_circles[3 + 3 * i];
    const double cy = m_circles[4 + 3 * i];
    const double r  = m_circles[5 + 3 * i];
    const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

    Point p;
    
    p.m_x = cx + r * (x - cx) / d;
    p.m_y = cy + r * (y - cy) / d;

    return p;
}

void RigidBodySimulator::AddToRobotConfiguration(int r, const double dx, const double dy, const double dtheta)
{
	//used 'sub_robot' instead of 'i' for the robot index here since 'i' is used in the for-loop
	m_robot.m_x[r] += dx;
	m_robot.m_y[r] += dy;
	m_robot.m_theta[r] += dtheta;
    
    const double ctheta = cos(m_robot.m_theta[r]);
    const double stheta = sin(m_robot.m_theta[r]);
    const int    n      = m_robot.m_currVertices[r].size();
    
    for(int i = 0; i < n; i += 2)
    {
	m_robot.m_currVertices[r][i] = 
	    ctheta * m_robot.m_initVertices[r][i] -
	    stheta * m_robot.m_initVertices[r][i + 1] + m_robot.m_x[r];
	
	m_robot.m_currVertices[r][i + 1] = 
	    stheta * m_robot.m_initVertices[r][i] +
	    ctheta * m_robot.m_initVertices[r][i + 1] + m_robot.m_y[r];
    }

}



void RigidBodySimulator::ReadRobot(const char fname[])
{
    FILE *in = fopen(fname, "r");
    int   n  = 0;
    
    if(in)
    {
	if(fscanf(in, "%d", &n) != 1)
	    return;
    
	m_robot.m_currVertices.resize(2 * n);	    
	for(int i = 0; i < 2 * n; ++i)
	    if(fscanf(in, "%lf", &(m_robot.m_currVertices[i])) != 1)
		return;
    
	m_robot.m_triangles.resize(3 * (n - 2));
	for(int i = 0; i < (int) m_robot.m_triangles.size(); ++i)
	    if(fscanf(in, "%d", &(m_robot.m_triangles[i])) != 1)
		return;
	    else
		printf("%d ", m_robot.m_triangles[i]);
	printf("\n...done with tris\n");
	
    
  
	m_robot.m_initVertices.assign(m_robot.m_currVertices.begin(),
				      m_robot.m_currVertices.end());
	fclose(in);	    
    }	
    else
	printf("..could not open file <%s>\n", fname);    
}



