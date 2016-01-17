using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Layering;

/// <summary>
///A wrapper for a 2D array that dynamically expands and refreshes the coordinates of the contents of the map
///This allows you to add contents to the map without worrying about resizing
/// </summary>
public class ExpandingMap : MonoBehaviour
{
	
	/// <summary>
	///The array containing all chunks within this map 
	/// </summary>
	public ChunkSet[] map;
	
	/// <summary>
	///The width of this map in chunks 
	/// </summary>
	public int currentWidth;
	/// <summary>
	///The height of this map in chunks 
	/// </summary>
	public int currentHeight;
	
	public override string ToString(){
		
		string s = "Width: " + currentWidth + " Height: " + currentHeight + "\n";
		
		for(int y = 0; y < currentHeight; y++){
			for(int x = 0; x < currentWidth; x++){
				
				int index = y * currentWidth + x;
				
				if(map[index] == null){
					s += "null,";
				}
				else{
					s += map[index].ToString();
				}
			}
			
			s += '\n';
		}
		
		return s;
		
	}
	
	/// <summary>
	///Returns the chunk at these coordinates 
	/// </summary>
	/// <param name="x">
	///The x coordinate of the target chunk
	/// </param>
	/// <param name="y">
	///The y coordinate of the target chunk
	/// </param>
	/// <returns>
	///The chunk at the target coordinates, if one exists
	/// </returns>
	public ChunkSet GetNodeAt(int x, int y){
		
		if(x < 0 || x >= currentWidth){
			return null;
		}
		
		if(y < 0 || y >= currentHeight){
			return null;
		}
		
		int index = y * currentWidth + x;
		
		if(index >= map.Length){
			return null;
		}
		
		return map[index];
		
	}
	
	/// <summary>
	///Adds a chunk at the given coordinates and expands the map 
	/// </summary>
	/// <param name="node">
	///The chunk you wish to add to the map
	/// </param>
	/// <param name="x">
	///The x target coordinate
	/// </param>
	/// <param name="y">
	///The y target coordinate
	/// </param>
	public void AddNodeAt(ChunkSet node, int x, int y){
				
		node.SetCoordinates(x,y);
		
		if(x < 0){
			
			//Debug.Log("ABS X: " + x);
			
			//expand and shuffle by Mathf.Abs(x)
			ExpandWidth(currentWidth + Mathf.Abs(x));
			ShuffleMap(Mathf.Abs(x),0);
			
			node.SetCoordinates(0,node.GetY());
		}
		else if(x >= currentWidth){
			//we just need to expand out by x - (currentWidth - 1), no need to shuffle
			
			if(currentWidth == 0){
				ExpandWidth(x+1);
			}
			else{
				ExpandWidth(x+1);
			}
		}
				
		if(y < 0){
			
			//Debug.Log("ABS Y: " + x);
			
			//expand and shuffle by Mathf.Abs(y)
			ExpandHeight(currentHeight + Mathf.Abs(y));
			ShuffleMap(0,Mathf.Abs(y));
			node.SetCoordinates(node.GetX(),0);
			
		}
		else if(y >= currentHeight){
			//we just need to expand up by y - (currentHeight - 1), no need to shuffle
			ExpandHeight(y+1);
		}
		
		SetNodeAt(node,node.GetX(),node.GetY());
				
	}
	
	void SetNodeAt(ChunkSet node, int x, int y){
				
		int index = y * currentWidth + x;
		
		map[index] = node;
		
		node.SetCoordinates(x,y);
	}
	
	void ExpandWidth(int newWidth){
		
		ChunkSet[] newMap = new ChunkSet[currentHeight * newWidth];
	
		if(map != null){
			
			int upperBound = currentWidth;
			
			if(currentWidth > newWidth){
				upperBound = newWidth;
			}
			
			for(int x = 0; x < upperBound; x++){
				for(int y = 0; y < currentHeight; y++){
					
					int index = y * currentWidth + x;
					
					ChunkSet n = map[index];
					
					index = y * newWidth + x;
					
					newMap[index] = n;
				}
			}
			
		}
		
		map = newMap;
		
		currentWidth = newWidth;
	}
	
	void ExpandHeight(int newHeight){
		
		ChunkSet[] newMap = new ChunkSet[newHeight * currentWidth];
	
		if(map != null){
			
			int upperBound = currentHeight;
			
			if(currentHeight > newHeight){
				upperBound = newHeight;
			}
			
			//When expanding height, we don't need to do any tricky mathematics to cater for one-dimensional array storage
			for(int x = 0; x < currentWidth; x++){
				for(int y = 0; y < upperBound; y++){
					
					int index = y * currentWidth + x;
					
					ChunkSet n = map[index];
					
					newMap[index] = n;
				}
			}
			
		}
		map = newMap;
		
		currentHeight = newHeight;
	
	}
	
	void ShuffleMap(int x_direction, int y_direction){
		
		//shuffle x
		if(x_direction != 0){
			
			int x_increment;
			int x_start;
			int x_end;
			
			if(x_direction > 0){
				x_increment = -1;	
				x_start = currentWidth-1;
				x_end = 0;
			}
			else{
				x_increment = 1;
				x_start = 0;
				x_end = currentWidth-1;
			}
			
			for(int x = x_start; x != x_end; x += x_increment){
			
				for(int y = 0; y < currentHeight; y++){
											
					int to_index = y * currentWidth + x;
										
					if(x - x_direction >= currentWidth || x - x_direction < 0){
						
						continue;
					}
					
					int from_index = y * currentWidth + (x - x_direction);
										
					map[to_index] = map[from_index];
					
					if(map[to_index] != null){
						map[to_index].SetCoordinates(x,y);
					}
					
				}
			}
			
			for(int y = 0; y < currentHeight; y++){
				int index = y * currentWidth + x_end;
				
				map[index] = null;
			}
		}
		
		//shuffle y
		if(y_direction != 0){
			
			int y_increment;
			int y_start;
			int y_end;
			
			if(y_direction > 0){
				y_increment = -1;	
				y_start = currentHeight-1;
				y_end = 0;
			}
			else{
				y_increment = 1;
				y_start = 0;
				y_end = currentHeight-1;
			}
			
			for(int y = y_start; y != y_end; y += y_increment){
			
				for(int x = 0; x < currentWidth; x++){
					
					int to_index = y * currentWidth + x;
										
					if(y - y_direction >= currentHeight || y - y_direction < 0){
						
						continue;
					}
					
					int from_index = (y - y_direction) * currentWidth + x;
										
					map[to_index] = map[from_index];
					
					if(map[to_index] != null){
						map[to_index].SetCoordinates(x,y);
					}
					
				}
				
			}
			
			for(int x = 0; x < currentWidth; x++){
				int index = y_end * currentWidth + x;
				
				map[index] = null;
			}
		}
	}
	
}

/*public interface MapChunk{

	void SetCoordinates(int x, int y);
	int GetX();
	int GetY();
	
}*/

