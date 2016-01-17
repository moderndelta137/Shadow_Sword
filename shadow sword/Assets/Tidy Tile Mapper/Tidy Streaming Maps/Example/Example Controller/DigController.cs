using UnityEngine;
using System.Collections;
using DopplerInteractive.TidyTileMapper.Utilities;

public class DigController : PlatformerPlayer {
	
	public string digButton = "Fire1";
	StreamingMap streamingMap;
	
	enum Direction{
		None,
		Left,
		Right,
		Up,
		Down
	}
		
	public override void OnInitializeEntity ()
	{
		base.OnInitializeEntity ();
		
		this.streamingMap = GameObject.FindObjectOfType(typeof(StreamingMap)) as StreamingMap;
	}
	
	public override void HandleInput (float deltaTime)
	{
		base.HandleInput (deltaTime);
		
		//We want to use the basic input, but we also want to observe a few extra keys
		if(Input.GetButtonDown(digButton)){
			
			//Dig
			Dig();
			
		}
	}
	
	public void Dig(){
		
		//Set the facing direction
		
		Direction facing = Direction.None;
		
		//You'll need to change this if you're using some sort of smoothing
		//this is just to show the dealio
		Vector3 forward = transform.forward;
		
		if(forward == frontDirection){
			facing = Direction.Down;
		}
		else if(forward == backDirection){
			facing = Direction.Up;
		}
		else if(forward == leftDirection){
			facing = Direction.Left;
		}
		else if(forward == rightDirection){
			facing = Direction.Right;
		}
		
		int d_x = y,d_y = x;
		
		switch(facing){
			case Direction.Down:{
			
				d_x = this.x;
				d_y = this.y + 1;
				break;
			}
			case Direction.Up:{
			
				d_x = this.x;
				d_y = this.y - 1;
				break;
			}
			case Direction.Right:{
			
				d_x = this.x+1;
				d_y = this.y;
				break;
			}
			case Direction.Left:{
			
				d_x = this.x-1;
				d_y = this.y;
				break;
			}
			case Direction.None:{
				return;
			}
		}
		
		//We need to do a few things here
		if(streamingMap != null){
			if(d_x < 0 || d_x >= streamingMap.width
				|| d_y < 0 || d_y >= streamingMap.height){
				
				return;
				
			}
		}
		else{
			
			if(d_x < 0 || d_x >= BlockUtilities.GetMapWidth(parentMap)
				|| d_y < 0 || d_y >= BlockUtilities.GetMapHeight(parentMap)){
				
				return;
				
			}
			
		}
		
		//Looks like we can do it!
		//You will probably want to implement a "Diggable" / "Not Diggable" logic here
		//But for now - we dig!
		
		BlockUtilities.RemoveBlockFromMap(parentMap,d_x,d_y,0,false,false);
		
		if(streamingMap != null){
			streamingMap.SetBlockPrefabAt(null,d_x,d_y,0);	
		}
		
		
	}
		
}
