using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DopplerInteractive.TidyTileMapper.Utilities.Pathing
{
	public class PathFinding
	{
		static List<PathNode> path = new List<PathNode>();
		static List<PathNode> closed = new List<PathNode>();
		static List<PathNode> open = new List<PathNode>();
		
		//static PathNode[] allNodes;
		//static int width;
		//static int height;
		
		public static float acceptablePathingTime = 100.0f;
		
		/*public static void InitializeToMap(IPathMap map){
			
			allNodes = new PathNode[map.GetMapWidth() * map.GetMapHeight()];
			
			width = map.GetMapWidth();
			height = map.GetMapHeight();
			
			for(int x = 0; x < width; x++){
				for(int y = 0; y < height; y++){
			
					int index = y * width + x;		
					
					allNodes[index] = new PathNode(x,y);
					
				}	
			}
			
		}*/
		
		static PathNode GetNodeFor(int x, int y, int dx, int dy){
					
			PathNode p = new PathNode(x,y,dx,dy);
			
			return p;
		}
		
		static PathNode GetNodeFor(int x, int y){
		
			PathNode p = new PathNode(x,y);
			
			return p;
		}
			
		static DateTime startPathTime;
		static DateTime endPathTime;
		
		static int[] weightMap;
		
		/// <summary>
		///Returns a list of PathNodes (x,y coordinates) representing a 2D path between 2 points 
		/// </summary>
		/// <param name="x1">
		///The x coordinate of point A
		/// </param>
		/// <param name="y1">
		///The y coordinate of point A
		/// </param>
		/// <param name="x2">
		///The x coordinate of point B
		/// </param>
		/// <param name="y2">
		///The y coordinate of point B
		/// </param>
		/// <param name="pathMap">
		///The map over which to path
		/// </param>
		/// <param name="depth">
		///The depth at which to path over the map
		/// </param>
		/// <param name="allowDiagonals">
		///Should the path allow diagonals?
		/// </param>
		/// <param name="randomiseWeightMap">
		///Should the weights between tiles be randomised? (Generates an imperfect path)
		/// </param>
		/// <param name="randomisationAmount">
		///By how much should the weights between tiles be randomised?
		/// </param>
		/// <param name="pathOverAllBlocks">
		///Should we disregard CanWalkAt() results and path over everything?
		/// </param>
		/// <returns>
		///A list of PathNodes, representing the path. Null or empty if no path available.
		/// </returns>
	    public static List<PathNode> GetPath(int x1, int y1, int x2, int y2, IPathMap pathMap, int depth, bool allowDiagonals, bool randomiseWeightMap, int randomisationAmount, bool pathOverAllBlocks)
	    {
			if(randomiseWeightMap){
				
				weightMap = new int[pathMap.GetMapWidth() * pathMap.GetMapHeight()];
				
				string w = "";
				
				for(int i = 0; i < weightMap.Length; i++){
					weightMap[i] = UnityEngine.Random.Range(0,randomisationAmount);
					w+=weightMap[i]+",";
				}
				
			}
			
			startPathTime = DateTime.Now;
			
			if(x1 < 0 || x1 >= pathMap.GetMapWidth()){
				return null;
			}
			
			if(y1 < 0 || y1 >= pathMap.GetMapHeight()){
				return null;
			}
			
			if(x2 < 0 || x2 >= pathMap.GetMapWidth()){
				return null;
			}
			
			if(y2 < 0 || y2 >= pathMap.GetMapHeight()){
				return null;
			}
			
	        PathNode start = GetNodeFor(x1, y1, x2, y2);
	        PathNode end = GetNodeFor(x2, y2, x2, y2);
	
	        path.Clear();
			closed.Clear();
			open.Clear();
	
	        PathNode current = start;
	
	        open.AddRange(GetValidTiles(closed, open, current, pathMap, end, depth,allowDiagonals,randomiseWeightMap, randomisationAmount,pathOverAllBlocks));
	
	        while (open.Count > 0 && !current.Equals(end))
	        {
	            RemoveNodeFromList(open,current);
	
	            closed.Add(current);
	
	            open.AddRange(GetValidTiles(closed, open, current, pathMap, end, depth,allowDiagonals,randomiseWeightMap, randomisationAmount,pathOverAllBlocks));
	
	            current = GetBestTile(open);
				
				endPathTime = DateTime.Now;
				
				if((endPathTime - startPathTime).TotalMilliseconds >= acceptablePathingTime){
					Debug.LogWarning("Pathing from: " + x1 + "," + y1 + " to " + x2 + "," + y2 + " took " + (endPathTime - startPathTime).TotalMilliseconds + "ms.\n" +
					"Total open: " + open.Count + " Total closed: " + closed.Count + " exceeded acceptable pathing time.");
					
					break;
				}
	        }
	
	        if (current == null)
	        {
	            return null;
	        }
	
	        while (current != null && !current.Equals(start) && current.cameFrom != null)
	        {
	            if (current != end)
	            {
	                path.Insert(0, current);
	            }
	
	            current = current.cameFrom;
	            
	        }
			
			
			
	        return path;
	
	    }
	
	    static PathNode GetBestTile(List<PathNode> nodes)
	    {
	        int index = -1;
	        float lowest = Int16.MaxValue;
	
	        for (int i = 0; i < nodes.Count; i++)
	        {
	            if (nodes[i].h < lowest)
	            {
	                index = i;
	                lowest = nodes[i].h;
	            }
	        }
	
	        if (index == -1)
	            return null;
	
	        return nodes[index];
	    }
		
		static List<PathNode> nodes = new List<PathNode>();
		
	    static List<PathNode> GetValidTiles(List<PathNode> closedNodes, List<PathNode> openNodes, PathNode node, IPathMap pathMap, PathNode endNode, int depth,
		                                    bool allowDiagonals,bool randomiseWeightMap, int randomisationAmount, bool pathOverAllBlocks)
	    {
	        int r_x = node.x;
	        int r_y = node.y;
	
	        nodes.Clear();
	
	        //the maximum nodes would be 8
	        //u,d,l,r,ul,ur,dl,dr
	        //PathNode[] nodes = new PathNode[8];
	        //int nodeIndex = 0;
	
	        for (int x = r_x - 1; x <= r_x + 1; x++)
	        {
	            if (x >= pathMap.GetMapWidth() || x < 0)
	                continue;
	
	            for (int y = r_y - 1; y <= r_y + 1; y++)
	            {
	                if (y >= pathMap.GetMapHeight() || y < 0)
	                    continue;
	                
					if(!allowDiagonals){
	                	if (x != r_x && y != r_y){
	                    	continue;
						}
					}
	
	                //the target won't be walkable. guarantee
	                if (x == endNode.x && y == endNode.y)
	                {
	                    //nodes[nodeIndex] = endNode;
	                    //Debug.Log("Added endNode");
	                    endNode.cameFrom = node;
	                    nodes.Add(endNode);
	                    break;
	                }
					
					PathNode p = GetNodeFor(x, y, endNode.x, endNode.y);
	
	                if (ListContains(closedNodes,p))
	                {
	                    continue;
	                }
	
	                if (ListContains(openNodes,p))
	                {
	                    continue;
	                }
					
	                if (!pathMap.IsWalkable(x, y, depth, pathOverAllBlocks))
	                {
	                    continue;
	                }
	
	                p.cameFrom = node;
	
	                nodes.Add(p);
	
					if(randomiseWeightMap){
						p.h += weightMap[y * pathMap.GetMapWidth() + x];
					}
					
	                //nodes[nodeIndex] = p;
	                //nodeIndex++;
	            }
	        }
	
	        //PathNode[] returnNodes = new PathNode[nodeIndex];
	        //for (int i = 0; i < nodeIndex; i++)
	        //{
	            //returnNodes[i] = nodes[i];
	        //}
	
	        //return returnNodes;
	
	        return nodes;
	    }
		
		static void RemoveNodeFromList(List<PathNode> list, PathNode p){
		
			int length = list.Count;
			
			for(int i = 0; i < length; i++){
				
				if(list[i].Equals(p)){
					list.RemoveAt(i);
					return;
				}
				
			}
		
		}
		
		static bool ListContains(List<PathNode> list, PathNode p){
			
			int length = list.Count;
			
			for(int i = 0; i < length; i++){
				
				if(list[i].Equals(p)){
					return true;
				}
				
			}
			
			return false;
			
		}
	}
}

