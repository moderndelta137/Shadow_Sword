using UnityEngine;
using System.Collections;
using DopplerInteractive.TidyTileMapper.Utilities;
using System.Collections.Generic;

public class StreamingMap : MonoBehaviour{
	
	//A 2D map (faux 2D) holding the prefabs of blocks in the map
	//If we wish to change a block, we should change this
	public StreamingMapNode[] nodeMap;
	
	//The width/height/depth of the prefab map
	//[HideInInspector]
	public int width;
	public int height;
	public int depth;
	
	//The blockmap that is created at runtime
	public BlockMap blockMap;
	
	//The current point of focus (the character's coordinates)
	public int focus_x;
	public int focus_y;
	public int focus_z;
	
	//I really can't remember what this does right now.
	Vector3 vCoords;
	
	//The radius within which blocks will be drawn
	public int drawRadius;
	
	//We use HashSets for our lists of blocks to be created and destroyed
	//and the list of blocks that is currently in focus
	//They have O(1) access time, which is blazingly fast
	HashSet<Vector3> currentlyInFocus = new HashSet<Vector3>();
	
	HashSet<Vector3> toBeCreated = new HashSet<Vector3>();
	HashSet<Vector3> toBeDestroyed = new HashSet<Vector3>();
	
	HashSet<Vector3> toBeCreated_Background = new HashSet<Vector3>();
	HashSet<Vector3> toBeDestroyed_Background = new HashSet<Vector3>();
	
	//We spread our destroy/create functions over multiple frames
	//in order to make it even more delicious and efficient
	
	//This indicates how often the delete / create function should be called.
	//0.0f = every frame
	float deleteRate = 0.0f;
	float createRate = 0.0f;
	float lastDelete = 0.0f;
	float lastCreate = 0.0f;
	
	//This indicates how many times the delete / create function should be called per call
	int actionsPerPass = 3;
	
	public bool generateMapOnAwake = false;
			
	//We sort our lists according to proximity to the player
	//In this manner we do all we can to avoid "Popping"
	void SortAllCollections(){
		
		List<Vector3> tbc = new List<Vector3>(toBeCreated);
		tbc.Sort(new CreateComparer(this));
		toBeCreated = new HashSet<Vector3>(tbc);
		
		List<Vector3> tbd = new List<Vector3>(toBeDestroyed);
		tbd.Sort(new DestroyComparer(this));
		toBeDestroyed = new HashSet<Vector3>(tbd);
	}
	
	//Initialize the prefab map (create the array)
	public void InitializePrefabMap(int width, int height, int depth){

		if(nodeMap == null || nodeMap.Length == 0){
	
			this.width = width;
			this.height = height;
			this.depth = depth;
			
			nodeMap = new StreamingMapNode[width * height * depth];
		}
	}
		
	public void ClearPrefabMap(){
		nodeMap = null;
	}
	
	//Initialize the streaming map (create the prefab map, enable pooling)
	void InitializeStreamingMap(int width, int height, int depth){
		
		InitializePrefabMap(width,height,depth);
		
		AssetPool.EnablePooling();
	}
	
	public Vector3 GetItemFromSet(HashSet<Vector3> hashSet){
		foreach(Vector3 v in hashSet){
			return v;
		}
		
		return Vector3.zero;
	}
	
	public List<Vector3> ListFromSet(HashSet<Vector3> hashSet){
		
		List<Vector3> vs = new List<Vector3>();
		
		foreach(Vector3 v in hashSet){
			vs.Add(v);
		}
		
		return vs;
	}
		
	public int GetBlockVariantAt(int x, int y, int z){
		
		int index = GetIndexFor(x,y,z);
		
		if(index < 0 || index >= nodeMap.Length){
						
			return 0;
		}
		
		
		if(nodeMap[index] == null){
			return 0;
		}
		
		return nodeMap[index].variantIndex;
		
	}
	
	public StreamingMapNode GetNodeAt(int x,int y, int z){
		
		int index = GetIndexFor(x,y,z);
				
		if(index < 0 || index >= nodeMap.Length){
						
			return null;
		}
		
		
		if(nodeMap[index] == null){
			return null;
		}
		
		return nodeMap[index];
		
	}
	
	//Retrieve the prefab object at the given coordinates
	//We need to generate our index as it is a 1D array
	public Block GetBlockPrefabAt(int x, int y, int z){
			
		int index = GetIndexFor(x,y,z);
				
		if(index < 0 || index >= nodeMap.Length){
						
			return null;
		}
		
		
		if(nodeMap[index] == null){
			return null;
		}
		
		return nodeMap[index].blockPrefab;
		
	}
	
	int GetIndexFor(int x, int y, int z){
		return 	x+y*width+(z*width*depth);
	}
	
	//Is this coordinate within our radius?
	bool IsWithinRadius(Vector3 focus, Vector3 target){
		
		float dist = Mathf.Abs(Vector3.Distance(target,focus));
							
		float n = 1.0f - ((dist) / (float)(drawRadius+1.0f));
		
		if(n < 0.0f){
			return false;
		}
		
		return true;
		
	}
	
	//Is this coordinate on the outer rim of the radius?
	bool IsOnOuterRim(Vector3 focus, Vector3 target){
		
		float dist = Mathf.Abs(Vector3.Distance(target,focus));
							
		float n = 1.0f - ((dist) / (float)(drawRadius+1.0f));
		
		if(n == 0.0f){
			return true;
		}
		
		return false;
		
	}
	
	public void SetBlockNodeAt(StreamingMapNode node, int x, int y, int z){
		
		int index = GetIndexFor(x,y,z);
		
		if(index < 0 || index >= nodeMap.Length){
						
			return;
		}
		
		nodeMap[index] = node;
		
	}
	
	public void SetVariantAt(int variant, int x, int y, int z){
		
		int index = GetIndexFor(x,y,z);
		
		if(index < 0 || index >= nodeMap.Length){
						
			return;
		}
		
		if(nodeMap[index] != null){
			nodeMap[index].variantIndex = variant;
		}
		
	}
	
	//Set the prefab at this coordinate to... the desired prefab
	public void SetBlockPrefabAt(Block blockPrefab, int x, int y, int z){
		
		int index = GetIndexFor(x,y,z);
				
		if(index < 0 || index >= nodeMap.Length){
						
			return;
		}
				
		if(nodeMap[index] == null){
						
			nodeMap[index] = new StreamingMapNode(blockPrefab);
		}
		else{
			
			nodeMap[index].blockPrefab = blockPrefab;
		}
		
	}
	
	//Set the prefab at this coordinate to... the desired prefab
	public void SetBlockPrefabAt(Block blockPrefab, int variant, int x, int y, int z){
		
		int index = GetIndexFor(x,y,z);
		
		if(index < 0 || index >= nodeMap.Length){
						
			return;
		}
		
		if(nodeMap[index] == null){
			nodeMap[index] = new StreamingMapNode(blockPrefab,variant);
		}
		else{
			nodeMap[index].blockPrefab = blockPrefab;
			nodeMap[index].variantIndex = variant;
		}
		
	}
	
	public void SetBackgroundAt(Material m, int x, int y, int z){
		
		
		int index = GetIndexFor(x,y,z);
		
		if(index < 0 || index >= nodeMap.Length){
						
			return;
		}
		
		if(nodeMap[index] == null){
			nodeMap[index] = new StreamingMapNode(m);
		}
		else{
			nodeMap[index].backgroundEntry = m;
		}		

	}
	
	public void OutputPrefabMap(){
		
		string s = "";
		
		for(int x = 0; x < width; x++){
			for(int y = 0; y < height; y++){
				
				Block b = GetBlockPrefabAt(x,y,1);
				
				s += (b != null ? b.name : "Null")[0] + ",";
			}
			
			s += "\n";
		}
		
		Debug.Log(s);
	}
	
	//Generate our map
	public void GenerateMap(int width, int height, int depth,
		string mapName, Vector3 tileScale, 
		int chunkWidth, int chunkHeight, 
		BlockMap.GrowthAxis growthAxis,
		float deleteUpdateRate,float createUpdateRate,int actionsPerPass){
				
		this.deleteRate = deleteUpdateRate;
		this.createRate = createUpdateRate;
		this.actionsPerPass = actionsPerPass;
		
		if(this.blockMap == null){
			
			InitializeStreamingMap (width,height,depth);
			
			this.blockMap = BlockUtilities.CreateBlockMap(mapName,tileScale,chunkWidth,chunkHeight,growthAxis);
						
		}
		
	}
	
	//This is the meat of the function
	//It indicates to the map that it is time to redraw given the current focus
	//within the given radius
	//If bypass queue is true - it will not add the action to the staggered queue of actions,
	//but instead just do it right now (call this on the first frame from your character)
	public void DrawMap(int focus_x, int focus_y, int focus_z, int radius, bool bypassQueue){
		
		if(blockMap == null){
			Debug.LogWarning("No block map: aborting");
			return;
		}
		
		this.focus_x = focus_x;
		this.focus_y = focus_y;
		this.focus_z = focus_z;
		
		this.vCoords = new Vector3(focus_x,focus_y,focus_z);
		
		this.drawRadius = radius;
		
		Vector3 focus = new Vector3(focus_x,focus_y,focus_z);
				
		HashSet<Vector3> newFocus = new HashSet<Vector3>();
		
		for(int x1 = focus_x - radius; x1 <= focus_x + radius; x1++){
			
			
			if(x1 < 0 || x1 >= width){
				continue;
			}
			
			
			for(int y1 = focus_y - radius; y1 <= focus_y + radius; y1++){
				
				if(y1 < 0 || y1 >= height){
					continue;
				}
				
				
				for(int z1 = focus_z - radius; z1 <= focus_z + radius; z1++){
					
					if(z1 < 0 || z1 >= depth){
						continue;
					}
					
					Vector3 f = new Vector3(x1,y1,z1);
					
					if(IsWithinRadius(focus,f)){
						
						newFocus.Add (f);
						
					}
				}	
			}
		}
				
		foreach(Vector3 v in currentlyInFocus){
			if(!newFocus.Contains(v)){
				
				if(bypassQueue){
					DestroyBlock(v);
					RemoveBackgroundCoord(v);
				}
				else{
					DeleteBlock(v);
					RemoveBackgroundCoord(v);
				}
			}
		}
				
		foreach(Vector3 v in newFocus){
					
			if(!currentlyInFocus.Contains(v)){
								
				if(bypassQueue){
					InstantiateBlock(v);
					AddBackgroundCoord(v);
				}
				else{
					AddBlock(v);
					AddBackgroundCoord(v);
				}
				
			}
		}
		
		currentlyInFocus = newFocus;
		
		SortAllCollections();
		
	}
	
	//Update all of our queues!
	void Update(){
		UpdateDeletion (Time.deltaTime);
		UpdateCreation(Time.deltaTime);
		UpdateBackgrounds();
		
		if(backgroundChanged){
			
			blockMap.RecalculateBackground();
			
			backgroundChanged = false;
			
		}
	}
	
	void UpdateBackgrounds(){
				
		foreach(Vector3 v in toBeDestroyed_Background){
			RemoveBackgroundFor(v);
		}
		
		if(toBeCreated_Background.Count > 0){
			AddBackgroundFor(ListFromSet(toBeCreated_Background));
		}
		
		toBeDestroyed_Background.Clear();
		toBeCreated_Background.Clear();
	}
	
	//Update our delete queue, delete if our timer says to
	void UpdateDeletion(float deltaTime){
		lastDelete += deltaTime;

		if(lastDelete >= deleteRate){
		
			lastDelete = lastDelete - deleteRate;
			
			for(int i = 0; i < actionsPerPass; i++){
				if(toBeDestroyed.Count > 0){
								
					Vector3 v = GetItemFromSet(toBeDestroyed);
					
					DestroyBlock(v);
					toBeDestroyed.Remove(v);
				}
			}
			
		}

	}
	
	//Update our create queue, create if our timer says to
	void UpdateCreation(float deltaTime){
		lastCreate += deltaTime;
	
		if(lastCreate >= createRate){
		
			lastCreate = lastCreate - createRate;
			
			for(int i = 0; i < actionsPerPass; i++){
				if(toBeCreated.Count > 0){
					
					Vector3 v = GetItemFromSet(toBeCreated);
					
					InstantiateBlock(v);
					toBeCreated.Remove(v);
					
				}
			}
		}
	
	
	}
		
	//Add this block to the destroy queue
	void DeleteBlock(Vector3 coords){
		
		if(toBeDestroyed.Contains(coords)){
			return;
		}
		
		if(toBeCreated.Contains(coords)){
			toBeCreated.Remove(coords);
		}
		
		if(toBeCreated_Background.Contains(coords)){
			toBeCreated_Background.Remove(coords);
		}
		
		toBeDestroyed.Add(coords);
		toBeDestroyed_Background.Add(coords);
	}
	
	//Add this block to the create queue
	void AddBlock(Vector3 coords){
		
		if(toBeCreated.Contains(coords)){
			return;
		}
		
		if(toBeDestroyed.Contains(coords)){
			toBeDestroyed.Remove(coords);
		}
				
		toBeCreated.Add(coords);
	}
	
	void RemoveBackgroundCoord(Vector3 coords){
		
		if(toBeDestroyed_Background.Contains(coords)){
			return;
		}
				
		if(toBeCreated_Background.Contains(coords)){
			toBeCreated_Background.Remove(coords);
		}
		
		toBeDestroyed_Background.Add(coords);
	}
	
	void AddBackgroundCoord(Vector3 coords){
		
		if(toBeCreated_Background.Contains(coords)){
			return;
		}
		
		if(toBeDestroyed_Background.Contains(coords)){
			toBeDestroyed_Background.Remove(coords);
		}
		
		toBeCreated_Background.Add(coords);
	}
	
	void DestroyBlock(Vector3 coords){
		
			int x = (int)coords.x;
			int y = (int) coords.y;
			int z = (int) coords.z;
				
			Block b = BlockUtilities.GetBlockAt(blockMap,x,y,z);
			OrientedBlock ob = null;
				
			if(b != null){
				ob = b as OrientedBlock;
			}
		
			if(ob != null){
			
				StreamingMapNode n = GetNodeAt (x,y,z);
			
				if(n != null && !n.HasVariant()){
					n.variantIndex = ob.GetCurrentVariant();
				}
			
				GameObject cio = ob.GetCurrentInstantiatedObject();
			
				if(cio != null){
				
					TidyMapBoundObject mbo = cio.GetComponentInChildren<TidyMapBoundObject>();
				
					if(mbo != null){
						if(!mbo.DestroyWhenStreaming()){
							return;
						}
					}
					
				}
			}
		
      		BlockUtilities.AddBlockToMap(blockMap,null,false,0,true,x,y,z,false,false);
		
	}
	
	//Instantiate our block! Wrap the AssetPool functions nicely
	void InstantiateBlock(Vector3 coords){
		
		int x = (int)coords.x;
		int y = (int)coords.y;
		int z = (int)coords.z;
				
		StreamingMapNode n = GetNodeAt(x,y,z);
		
		Block b = null;
		int bv = 0;
		
		if(n != null){
			b = n.blockPrefab;
			bv = n.variantIndex;
		}
				
		Block toAdd = null;
		
		if(b != null){
			
			GameObject o = AssetPool.Instantiate(b.gameObject) as GameObject;
			
#if UNITY_4_0
			o.SetActive(true);
#else
			o.SetActiveRecursively (true);
#endif
			
			toAdd = o.GetComponent<Block>();
			
			OrientedBlock ob = toAdd as OrientedBlock;
			
			if(ob != null){
				ob.PreRandomiseBlockOrientations();
			}
			
		}
				
		if(n != null && n.HasVariant()){
			BlockUtilities.AddBlockToMap(blockMap,toAdd,false,bv,true,x,y,z,false,false);
		}
		else{
			BlockUtilities.AddBlockToMap(blockMap,toAdd,true,bv,true,x,y,z,false,false);
		}
		
		if(n != null){
						
			if(!n.HasVariant()){
				
				Vector3 focus = new Vector3(focus_x,focus_y,focus_z);
				
				if(!IsOnOuterRim(focus,coords)){
					//Let's save the variant so that we always get a consistent map
					//It saves the programmer having to randomise this themselves
					OrientedBlock ob = BlockUtilities.GetBlockAt(blockMap,x,y,z) as OrientedBlock;
					
					if(ob != null){
						
						n.variantIndex = ob.GetCurrentVariant();
						
						
					}
				}
				
			}
			
		}
		
	}
	
	bool backgroundChanged = false;
	
	void RemoveBackgroundFor(Vector3 coords){
		
		if(nodeMap == null || nodeMap.Length == 0){
			return;
		}
		
		int x = (int)coords.x;
		int y = (int)coords.y;
		int z = (int)coords.z;
				
		blockMap.RemoveFromBackground(new Vector3(x,y,z),false);
				
		backgroundChanged = true;
	}
	
	void AddBackgroundFor(List<Vector3> coords){
		
		if(nodeMap == null || nodeMap.Length == 0){
			return;
		}
		
		List<StreamingMapNode> nodes = new List<StreamingMapNode>();
		
		for(int i = 0; i < coords.Count; i++){
					
			int x = (int) coords[i].x;
			int y = (int) coords[i].y;
			int z = (int) coords[i].z;
						
			StreamingMapNode n = nodeMap[GetIndexFor(x,y,z)];
			
			if(n == null){
				coords.RemoveAt(i);
				i--;
				continue;
			}
			
			Material m = n.backgroundEntry;
			
			if(m == null){
				coords.RemoveAt(i);
				i--;
				continue;
			}
			
			nodes.Add(n);
			
		}
		
		while(nodes.Count > 0){
			
			StreamingMapNode n = nodes[0];
			
			List<Vector3> mNodes = new List<Vector3>();
			
			for(int j = 0; j < nodes.Count; j++){
				
				if(n.backgroundEntry == nodes[j].backgroundEntry){
					mNodes.Add(coords[j]);
					nodes.RemoveAt(j);
					coords.RemoveAt(j);
					j--;
				}
				
			}
			
			if(!blockMap.HasBackgroundEntryFor(n.backgroundEntry.name)){
				blockMap.AddBackground(n.backgroundEntry);
			}
			
			blockMap.AddEntryToBackground(mNodes.ToArray(),n.backgroundEntry.name);
			
		}
		
		backgroundChanged = true;
		
	}
	
	public BlockMap GetMap(){
		return blockMap;
	}
	
	public void SetMap(BlockMap map){
		this.blockMap = map;
		
		InitializeStreamingMap(
					BlockUtilities.GetMapWidth(map),
					BlockUtilities.GetMapHeight(map),
					BlockUtilities.GetMapUpperDepth(map) - BlockUtilities.GetMapLowerDepth(map) + 1
					);
	}
	
	//A few custom comparers that help us be ultra-efficient
	
	public class CreateComparer : IComparer<Vector3>{
	
		StreamingMap map;
		
		public CreateComparer(StreamingMap map){
			this.map = map;
		}
		
		#region IComparer[Vector3] implementation
		public int Compare (Vector3 x, Vector3 y)
		{
			float a = Mathf.Abs (Vector3.Distance(x,map.vCoords));
			float b = Mathf.Abs (Vector3.Distance(y,map.vCoords));
			
			return (int)(a - b);
		}
		#endregion
		
	}
	
	public class DestroyComparer : IComparer<Vector3>{
	
		StreamingMap map;
		
		public DestroyComparer(StreamingMap map){
			this.map = map;
		}
		
		#region IComparer[Vector3] implementation
		public int Compare (Vector3 x, Vector3 y)
		{
			float a = Mathf.Abs (Vector3.Distance(x,map.vCoords));
			float b = Mathf.Abs (Vector3.Distance(y,map.vCoords));
			
			return (int)(b - a);
		}
		#endregion
		
	}
	
	//Vaguely singleton-esque behaviour
	//For speed on map bound objects
	static StreamingMap instance;
	
	public static StreamingMap GetInstance(){
		
		if(instance == null){
			instance = GameObject.FindObjectOfType(typeof(StreamingMap)) as StreamingMap;
		}
		
		return instance;
		
	}
	
	//This is used when we have a game of multiple streaming maps over a few different scenes
	//Statics need to be cleared
	public static StreamingMap GetInstance(bool fresh){
		
		if(instance == null || fresh){
			instance = GameObject.FindObjectOfType(typeof(StreamingMap)) as StreamingMap;
		}
		
		return instance;
		
	}
}
