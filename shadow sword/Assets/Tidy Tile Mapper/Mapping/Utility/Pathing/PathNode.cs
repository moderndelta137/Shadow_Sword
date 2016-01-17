using System;
using System.Collections;

namespace DopplerInteractive.TidyTileMapper.Utilities.Pathing
{
	public class PathNode
	{
	    public int x;
	    public int y;
	
	    public float h;
	
	    public PathNode cameFrom = null;
	
	    public PathNode() { }
	
	    public PathNode(int x, int y)
	    {
	        this.x = x;
	        this.y = y;
	        this.h = 0.0f;
	    }
	
	    public PathNode(int x, int y, int goal_x, int goal_y)
	    {
	        this.x = x;
	        this.y = y;
	
	        this.h = GetDistance(x, y, goal_x, goal_y);
	    }
		
		public void SetDistance(int goal_x, int goal_y){
			this.h = GetDistance(x, y, goal_x, goal_y);
		}
		
	    public float GetDistance(int x, int y, int x2, int y2)
	    {
	        double a2 = Math.Pow(
	            (double)(x2 - x), (double)2)
	            + 
	            Math.Pow(
	            (double)(y2 - y), (double)2);
	
	        return (float)Math.Sqrt(a2);
	    }
	
	    public override string ToString()
	    {
	        return "Node: " + x + "," + y;
	    }
		
		public bool Equals(PathNode p){
			return (p.x == x && p.y == y);
		}
	
	    public override bool Equals(object obj)
	    {
	        if (obj.GetType() == this.GetType())
	        {
	            PathNode p = (PathNode)obj;
	
	            if (p.x == x && p.y == y)
	                return true;
	            return false;
	        }
	
	        return base.Equals(obj);
	    }
	
	    public override int GetHashCode()
	    {
	        return base.GetHashCode();
	    }
	}
}
