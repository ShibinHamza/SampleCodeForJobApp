
Vector2D computeAlignment(vector<Agent*> allAgents, double scan_radius) 
{
	Vector2D avgVel = Vector2D(0,0);
	int count = 0;		
	for(auto& agent: allAgents) 							// The for loop calculates the total velocity of all the agents within the scan radius from this ageny
	{
		if(agent==this) 							// Do not evaluate for the current agent
		{
			continue;
		} 
		else {
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
