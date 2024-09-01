class Agent : public Animat
{

public:
	//...........................................................
	// Constructor of the class
	// The agents are initiated at a random location and have 
	// sensors to detect angle and distance to the nearest agent.
	//...........................................................
	Agent()
    {        
	This.InitRandom = true;									
	This.Add("angle", NearestAngleSensor<Agent>());				// Nearest angle sensor to the nearest agent
	This.Add("Xdist", NearestXSensor<Agent>());					// This sensor outputs X distance to nearest agent
	This.Add("Ydist", NearestYSensor<Agent>());					// This sensor outputs Y distance to nearest agent
    }

	//..................................................................
	// Default control function overwritten for case specific algorithm
	//..................................................................
    virtual void Control()
    {
		////		SENSOR OUTPUTS STORED IN VARIABLE		////
		double xdist = This.Sensors["Xdist"]->GetOutput();
		double ydist = This.Sensors["Ydist"]->GetOutput();
		double angle_ = This.Sensors["angle"]->GetOutput();
		
		double dist = pow((pow(xdist,2)+pow(ydist,2)),0.5);			// Distance to nearest agent, calculated from sensor outputs
		double left_control, right_control;							// Variables to assign control signal to agent wheels
		
		////			agents is a vector of agents in the current world		////
		agents.clear();												// Clear the vector in each simulation	
		GetWorld().Get(agents);										// Assign the current world agent objects to the agents vector
	

		//
		// The alignment, cohesion vector stored to use in control 
		// signal formula
		//
		Vector2D alignment = This.computeAlignment(agents,scan_radius);			// Calculate Alignment vector
		Vector2D cohesion = This.computeCohesion(agents, scan_radius); 			// Calculate Cohesion vector
		
		////		COMPUTE SEPARATION		//// 
		double separation_angle;	
		double separationDistance = 0.05;

		//
		// If the distance to the nearest agent is above the separation distance, then this 
		// agent will move towards the nearest agent and is the distance is less then it 
		// will move away from the neighbour
		//
		if (dist > separationDistance) 
		{	
			separation_angle = angle_;
		} 
		else {
			
			// Below is an adjustment to get a standardized angle between -PI and +PI
			if (angle_<0) 
			{		
				separation_angle = angle_+3.14;
			}
			else {
				separation_angle = angle_-3.14;
			}
		
		}	

		//
		// Compute Cohesion and Alignment angles, derived from the 
		// alignment, cohesion vectors calculated earlier. This 
		// orientation value will be used in the formula for control
		// signal.	
		// 
		double align_angle= getAngle(alignment);
		double cohesion_angle = atan(cohesion.y / cohesion.x);

		//
		// Compute the required orientation of this animat
		// angleThis is a combination of the current orientation of the animat with
		// weighted contributions from alignment, cohesion, separation orientations
		//
		double angleThis;
		Vector2D velThis = this->GetVelocity();
		angleThis = getAngle(velThis)-align_weight*align_angle-cohesion_weight*cohesion_angle-separation_weight*separation_angle;

		//
		// Set values for right and left control
		// The control signal is calculated such that both left and right wheels 
		// will have the same default control signal defined by the def_ctrl variable
		// and when angleThis>0 the right control value will increase and when the 
		// value of angleThis<0 the left control value will increase. This enables a
		// change of orientation of the agent towards angleThis value with reference
		// to the current orientation of the agent.
		//
		double right_ctrl = def_ctrl + (angleThis>0?angleThis:0)*(def_ctrl/3.14) ;
		double left_ctrl = def_ctrl - (angleThis<0?angleThis:0)*(def_ctrl/3.14) ;
		

		// Assign the respective output to the control signal variables
		This.Controls["right"] = right_ctrl;
		This.Controls["left"] = left_ctrl;
	
    }   

	//....................................................................
	// This is called when an agent collides with any object in the World.
	//....................................................................
	virtual void OnCollision(WorldObject* obj)
	{
		Animat::OnCollision(obj);
	}

	//.............................................................
	//Function to compute alignment vector
	//.............................................................
	Vector2D computeAlignment(vector<Agent*> allAgents, double scan_radius) 
	{
		Vector2D avgVel = Vector2D(0,0);
		int count = 0;	

		//
		// Below for loop calculates the total velocity of all the agents within the 
		// scan radius from this agent
		//	
		for(auto& agent: allAgents) 
		{
			if(agent==this) 							// Do not evaluate for the current agent
			{
				continue;
			} 
			else {
				//
				// Computes total velocity of all other agents in the population
				// within the scan radius
				//
				if((this->GetLocation()-agent->GetLocation()).GetLength()<scan_radius) 
				{			
					avgVel = avgVel+agent->GetVelocity();
					count++;
				}
			}
		}
		
		if (count==0) 
		{
			return avgVel;
		} 
		else {
			avgVel = avgVel*((1.0f)/(double)count);					// Calculate average velocity
			double a = atan(avgVel.y/avgVel.x);						// Orientation of average velocity
			
			return avgVel;											// Average velocity of the neighbourhood is returned
		}

	}
