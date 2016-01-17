using UnityEngine;
using System.Collections;
using DopplerInteractive.TidyTileMapper.Utilities;

public class StreamingWorld : MonoBehaviour {
	
	StreamingMap streamingMap;
	
	public PlatformerPlayer playerPrefab;
	
	public int player_x;
	public int player_y;
	
	public int drawRadius = 10;
	
	PlatformerPlayer player;
	
	//Level generation
	
	//We'll generate some hills first
	public int minStartingHeight = 1;
	public int maxStartingHeight = 10;
	
	public int mapWidth = 20;
	public int mapHeight = 20;
	
	public int start_player_x;
	int start_player_y;
	
	public Block blockPrefab;
	public Material blockBackground;
	
	// Use this for initialization
	void Start () {
			
		//Sanity, please
		if(minStartingHeight <= 0){
			minStartingHeight = 1;
		}
		
		this.streamingMap = gameObject.AddComponent<StreamingMap>();
				
		streamingMap.GenerateMap(mapWidth,mapHeight,1,"Map",new Vector3(1.0f,1.0f,1.0f),5,5,BlockMap.GrowthAxis.Up,0,0,3);
		
		PopulateMap();
		AddPlayerToMap();
		
	}
	
	void PopulateMap(){
		
		GenerateHills();
		
		for(int x = 0; x < mapWidth; x++){
			
			for(int y = maxStartingHeight; y < mapHeight; y++){
								
				streamingMap.SetBlockPrefabAt(blockPrefab,0,x,y,0);
				streamingMap.SetBackgroundAt(blockBackground,x,y,0);
				
			}
			
		}
		
	}
	
	void GenerateHills(){
		
		//We will randomise the y value that we pass to the perlin function
		//This will result in a random level everytime
		float yOffset = UnityEngine.Random.value;
		
		//We do this at the top
		for(int x = 0; x < mapWidth; x++){
			
			//We'll get our perlin noise value -
			//normalizing our x to pass it as a parameter
			float p = Mathf.PerlinNoise((float)x/(float)maxStartingHeight,yOffset);
			
			//Given this value, we'll get our height cap
			//And set it to allow for an empty column at the top, and 
			//assure it doesn't leave gaps in the bottom
			int ny = (int)(p * (float)maxStartingHeight);
			
			//Given this, we'll set all values below this number to be 'true'
			for(int y = 0; y <= maxStartingHeight; y++){
				
				if(y >= ny){
					streamingMap.SetBlockPrefabAt(blockPrefab,x,y,0);
					streamingMap.SetBackgroundAt(blockBackground,x,y,0);
				}
				
			}
			
			if(x == start_player_x){
				
				start_player_y = ny-1;
				
			}
			
		}
			
		
	}
	
	void AddPlayerToMap(){
		
		player_x = start_player_x;
		player_y = start_player_y;
		
		streamingMap.DrawMap(player_x,player_y,1,drawRadius,true);
		
		//We want to draw the map before we add the player
		player = GameObject.Instantiate(playerPrefab) as PlatformerPlayer;
		
		Vector3 coords = BlockUtilities.GetMathematicalPosition(streamingMap.blockMap,player_x,player_y,0);
		
		player.transform.parent = streamingMap.blockMap.transform;
		
		player.transform.localPosition = coords;
		
		player.InitializeEntity(streamingMap.blockMap);
		
	}
	
	// Update is called once per frame
	void Update () {
	
		if(player.x != player_x || player.y != player_y){
			
			player_x = player.x;
			player_y = player.y;
			
			streamingMap.DrawMap(player_x,player_y,1,drawRadius,false);
			
		}
		
	}
}
