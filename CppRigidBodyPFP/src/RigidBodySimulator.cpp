#include "RigidBodySimulator.hpp"

RigidBodySimulator::RigidBodySimulator(void) //original/default constructor 
{
	//must use pushback-> insert single sub-robot configuration into super-robot at default configuration
	m_robot.m_x.push_back(0);
	m_robot.m_y.push_back(0);
	m_robot.m_theta.push_back(0);
	
	m_circles.push_back(16); //goal x
	m_circles.push_back(-7); //goal y
	m_circles.push_back(1.0); //goal radius
}

RigidBodySimulator::RigidBodySimulator(int number_of_bots) //
{
	//change to initialize every robot at a different location, positioned across the workspace at a uniform-random distribution
	/*NOTE: number of robots not known til ReadRobot is called,
	must create a way to find it by time of or on constructor call*/
	for(int i=0; i<number_of_bots; i++)
	{
		//workspace goes from (-22,-14) on bottom left to (22,14) on top right
		m_robot.m_x.push_back(pow(-1,rand()%2)*rand()%22); /*rand using x-range coordinates -22~22*/
		m_robot.m_y.push_back(pow(-1,rand()%2)*rand()%14); /*rand using y-range coordinates* -14~14*/
		/*should replace rand() above with function that returns a float/double instead of int for better implementation*/
		m_robot.m_theta.push_back(0);
		
		/**WILL ALSO NEED TO CHECK EACH NEW SUB-ROBOT FOR COLLISION BEFORE ADDING IT TO SUPER-ROBOT
		PROBABLY GENERATE PARAMETERS FIRST, CHECK FOR COLLISION, THEN
		IF NO COLLISION -> PUSH_BACK PARAMETERS, ELSE DO NOTHING*/
	}
	m_circles.push_back(16); //goal x
	m_circles.push_back(-7); //goal y
	m_circles.push_back(1.0); //goal radius
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

//new function
void RigidBodySimulator::ReadRobot(const char fname[]){
    FILE *in = fopen(fname, "r");
    
    int   n  = 0;
    int num;
    int i=0;
    int line[n]; //will store all vertices of the total #-of robots
    
    if(in){
		// n -should print the number of vertices read 
		
		if(fscanf(in, "%d", &n ) != 1)
			return;
		// n, stores the number of robots 
		printf("\nNumber of Robots: %d===========================>", n);
		
		//gets the line of vertices
		while( i != n && fscanf(in, "%d", &num) > 0  ){
			line[i] = num;	
			printf("\nRobot <%d> has <%d> Vertices", i, line[i]);
			i++;
		}
		
		//line[i] = n
    
		for (int j=0; j<n; j++){
			m_robot.m_currVertices.resize(2 * line[j]);
			
			printf("\nNumber of vertices <%d> ", line[j]);       
			
			for(i = 0; i < 2 * line[j]; i++){
				if(fscanf(in, "%lf", &(m_robot.m_currVertices[i])) != 1)	
					return;
					
				printf("\n %f", m_robot.m_currVertices[i] );
			}
			
			m_robot.m_triangles.resize(3 * (line[j] - 2) );
			printf("\nNumber of Triangles: %d\n",(int) m_robot.m_triangles.size());
			
			for(int k = 0; k < (int) m_robot.m_triangles.size(); ++k)
				if(fscanf(in, "%d", &(m_robot.m_triangles[k])) != 1)
					return;
				
				else
					printf(" %d ", m_robot.m_triangles[k]);
			
			 m_robot.m_initVertices.assign(	
									m_robot.m_currVertices.begin(),
									m_robot.m_currVertices.end());
			
			if(n>1){ //the txt file has more than one robot
				
			}
		
		}// end of FOR-LOOP
    
        fclose(in);            
        
    }        
    else 	printf("..could not open file <%s>\n", fname);    
}//end of READ-ROBOT




/*
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
*/

