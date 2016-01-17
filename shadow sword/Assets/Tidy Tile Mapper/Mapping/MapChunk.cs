using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Utilities;

/// <summary>
/// A map chunk - the smaller building block of a map (containing a small array of blocks)
/// </summary>
public class MapChunk : MonoBehaviour
{
	
	/// <summary>
	/// The blocks contained within this chunk (The number of blocks is width * height)
	/// </summary>
	public Block[] chunkPieces = null;
	
	/// <summary>
	/// The width and height of this chunk
	/// </summary>
	public int width, height;
	
	/// <summary>
	/// The coordinates of this chunk, in chunkmap coordinates
	/// </summary>
	public int x, y, depth;
	
	/// <summary>
	/// The map to which this chunk is bound
	/// </summary>
	public BlockMap parentMap;
	
	/// <summary>
	/// The cached root transform of this chunk
	/// </summary>
	public Transform chunkTransform = null;
	
	/// <summary>
	/// Returns the block contained at the target chunk coordinates (0...chunkWidth, 0...chunkHeight)
	/// </summary>
	/// <param name="x">
	///The target x chunk coordinate
	/// </param>
	/// <param name="y">
	///The target y chunk coordinate
	/// </param>
	/// <returns>
	///The block at this coordinate
	/// </returns>
	public Block GetBlockAtChunkCoord(int x, int y){
		
		if(chunkPieces == null){
			return null;
		}
		
		if(x < 0 || x >= parentMap.chunkWidth){
			//Debug.LogWarning("Fundamental mathematics problem attempting to get block " + x +"," + y + " in chunk: " + this.x + "," +this.y);
			return null;
		}
		
		if(y < 0 || y >= parentMap.chunkHeight){
			//Debug.LogWarning("Fundamental mathematics problem attempting to get block " + x +"," + y + " in chunk: " + this.x + "," +this.y);
			return null;
		}
		
		int index = y * parentMap.chunkWidth + x;
		
		if(index >= chunkPieces.Length){
			return null;
		}
		
		return chunkPieces[index];
	}
	
	/// <summary>
	/// Refreshes this chunk and the blocks contained within it.
	/// </summary>
	public void RefreshChunk(){
		
		if(chunkPieces == null || chunkPieces.Length <= 0){
			return;
		}
		
		for(int i = 0; i < chunkPieces.Length; i++){
			
			if(chunkPieces[i] != null){
				chunkPieces[i].RefreshBlock();
			}
			
		}
		
	}
		
	/// <summary>
	/// Sets the block at the target chunk coordinate
	/// </summary>
	/// <param name="x">
	///The target chunk x coordinate
	/// </param>
	/// <param name="y">
	///The target chunk y coordinate
	/// </param>
	/// <param name="b">
	///The block to add to the target chunk coordinate
	/// </param>
	public void SetBlockAt(int x, int y, Block b, bool destroyImmediate){
		
		if(chunkPieces == null){
			Debug.LogWarning("Block array in chunk: " + this.x + "," + this.y + " has not been initialized.");
			return;
		}
		
		if(x < 0 || x >= parentMap.chunkWidth){
			Debug.LogWarning("Fundamental mathematics problem attempting to set block " + x +"," + y + " in chunk: " + this.x + "," +this.y);
			return;
		}
		
		if(y < 0 || y >= parentMap.chunkHeight){
			Debug.LogWarning("Fundamental mathematics problem attempting to set block " + x +"," + y + " in chunk: " + this.x + "," +this.y);
			return;
		}
		
		int index = y * parentMap.chunkWidth + x;
		
		if(index >= chunkPieces.Length){
			Debug.LogWarning("Index exceeded array length in chunk: " + x + "," + y +". Index: " + index + " Length: " + chunkPieces.Length);
			return;
		}
		
		InitializeBlockPlacement(x,y,b);
		
		if(chunkPieces[index] != null){
			
			//Block cb = chunkPieces[index];
			
			if(AssetPool.IsPoolingEnabled()){
								
				AssetPool.Destroy(chunkPieces[index].gameObject);
				chunkPieces[index] = null;
				//cb = null;
				
			}
			else{
				if(destroyImmediate){
					GameObject.DestroyImmediate(chunkPieces[index].gameObject);
					chunkPieces[index] = null;
				}
				else{
					GameObject.Destroy(chunkPieces[index].gameObject);
					chunkPieces[index] = null;
				}
			}
			
		}
		
		chunkPieces[index] = b;
		
	}
	
	public void Editor_ActivateBound(int width, int height, int x, int y, int depth, BlockMap parentMap, MapChunk boundChunk){
		
		this.parentMap = parentMap;
		this.width = width;
		this.height = height;
		
		this.x = x;
		this.y = y;
		this.depth = depth;
		
		int chunkDiff = depth - boundChunk.depth;
		
		float heightOffset = 0.0f;
		
		if(chunkDiff == 1){
			//above
			heightOffset = parentMap.tileScale.z;
			
		}
		else if(chunkDiff == -1){
			//below
			heightOffset = -parentMap.tileScale.z;
		}
		else{
			Debug.LogWarning("Chunk depth difference inconsistent! Aneurysm imminent!");
		}
		
		transform.parent = parentMap.transform;
		
		Vector3 pos = Vector3.zero;
		
		if(parentMap.growthAxis == BlockMap.GrowthAxis.Up){
					
			pos = new Vector3(
			                          boundChunk.transform.localPosition.x,
			                          boundChunk.transform.localPosition.y,
			                          boundChunk.transform.localPosition.z + heightOffset
			                          );
			
			transform.localRotation = Quaternion.identity;
			
		}
		else if(parentMap.growthAxis == BlockMap.GrowthAxis.Forward ){
		
			pos = new Vector3(
			                          boundChunk.transform.localPosition.x,
			                  		  boundChunk.transform.localPosition.y + heightOffset,
			                          boundChunk.transform.localPosition.z
			                          );
			
			transform.localRotation = Quaternion.Euler(
			                                           new Vector3(
			                                                       90.0f,
			                                                       0.0f,
			                                                       0.0f
			                                                       ));
			
		}
					
		transform.localPosition = pos;
		
		transform.localScale = new Vector3(
					                                   parentMap.tileScale.x * width,
					                                   parentMap.tileScale.y * height,
					                                   parentMap.tileScale.z
					                                   );
		
	}
	
	/// <summary>
	/// Activate the chunk at this coordinate - this comes before initializing
	/// </summary>
	/// <param name="width">
	///The width of this chunk in blocks
	/// </param>
	/// <param name="height">
	///The height of this chunk in blocks
	/// </param>
	/// <param name="parentMap">
	///The map to which this chunk should be bound
	/// </param>
	public void Editor_Activate(int width, int height, int x, int y, int depth, BlockMap parentMap, bool positionAbsolute){
		
		this.parentMap = parentMap;
		this.width = width;
		this.height = height;
		
		this.x = x;
		this.y = y;
		this.depth = depth;
		
		if(!positionAbsolute){
			
			for(int x1 = this.x-1; x1 <= this.x+1; x1++){
				for(int y1 = this.y-1; y1 <= this.y+1; y1++){
					
					if(x1 == x && y1 == y){
						continue;
					}
					
					if(parentMap.HasChunkAt(x1,y1,depth)){
						
						MapChunk chunk = parentMap.GetChunkAt(x1,y1,depth,false);
						
						int difference_x = x1 - this.x;
						int difference_y = y1 - this.y;
						
						Vector3 pos = Vector3.zero;
						
						transform.parent = parentMap.transform;
						
						transform.localPosition = pos;
						
						float diff_dist_x = difference_x * (parentMap.tileScale.x * width);
						float diff_dist_y = -(difference_y * (parentMap.tileScale.y * height));
						
						if(parentMap.growthAxis == BlockMap.GrowthAxis.Up){
						
							pos = new Vector3(
							                          chunk.transform.localPosition.x + diff_dist_x,
							                          chunk.transform.localPosition.y - diff_dist_y,
							                          (depth * parentMap.tileScale.z)
							                          );
							
							transform.localRotation = Quaternion.identity;
							
						}
						else if(parentMap.growthAxis == BlockMap.GrowthAxis.Forward ){
						
							pos = new Vector3(
							                          chunk.transform.localPosition.x + diff_dist_x,
							                  		  (depth * parentMap.tileScale.z),
							                          chunk.transform.localPosition.z - diff_dist_y
							                          );
							
							transform.localRotation = Quaternion.Euler(
							                                           new Vector3(
							                                                       90.0f,
							                                                       0.0f,
							                                                       0.0f
							                                                       ));
							
						}
								
						transform.localPosition = pos;
						
						transform.localScale = new Vector3(
						                                   parentMap.tileScale.x * width,
						                                   parentMap.tileScale.y * height,
						                                   parentMap.tileScale.z
						                                   );
						
						return;
					}
					
				}	
			}
		}
		
		Vector3 localPos = Vector3.zero;
		
		if(positionAbsolute){
			
			if(parentMap.growthAxis == BlockMap.GrowthAxis.Up){
				localPos = new Vector3(-x * width,-y*height,0.0f);			
			}
			else{
				//if(parentMap.growthAxis == BlockMap.GrowthAxis.Up){
				localPos = new Vector3(-x * width,0.0f,-y*height);			
				//}
			}
		}
		
		transform.parent = parentMap.transform;
		transform.localScale = new Vector3(
		                                   parentMap.tileScale.x * width,
		                                   parentMap.tileScale.y * height,
		                                   parentMap.tileScale.z
		                                   );
		
		transform.localPosition = localPos;
		
		if(parentMap.growthAxis == BlockMap.GrowthAxis.Up){
					
			transform.localRotation = Quaternion.identity;
			
		}
		else if(parentMap.growthAxis == BlockMap.GrowthAxis.Forward ){
		
			transform.localRotation = Quaternion.Euler(
			                                           new Vector3(
			                                                       90.0f,
			                                                       0.0f,
			                                                       0.0f
			                                                       ));
			
		}
		
	}
	
	//We are passed a collection of gameobjects that have already been instantiated
	//as we need to use Editor.InstantiatePrefab
	//to keep the links
	/// <summary>
	/// Initialize this chunk with the blocks that it contains
	/// </summary>
	/// <param name="chunkPieces">
	///An array of instantiated blocks
	/// </param>
	public void Editor_InitializeChunk(Block[] chunkPieces){
		
		if(Editor_IsInitialized()){
						
			Debug.Log("Trying to initialize: " + name);
			
			return;
		}
				
		chunkTransform = transform;
		
		transform.localScale = new Vector3(1.0f,1.0f,1.0f);
		
		//Remove the collider
		//Remove the renderer
		//Remove the Mesh Filter
		if(gameObject.GetComponent<Collider>() != null){
			gameObject.GetComponent<Collider>().enabled = false;
			GameObject.DestroyImmediate(gameObject.GetComponent<Collider>());
		}
		
		if(gameObject.GetComponent<Renderer>() != null){
			GameObject.DestroyImmediate(gameObject.GetComponent<Renderer>());
		
		}
		
		MeshFilter m = gameObject.GetComponent<MeshFilter>();
		
		if(m != null){
			GameObject.DestroyImmediate(m);
		}
		
		
		for(int x = 0; x < width; x++){
		
			for(int y = 0; y < height; y++){
			
				int index = y * width + x;
				
				Block b = chunkPieces[index];
								
				InitializeBlockPlacement(x,y,b);
				
			}
			
		}
		
		//Populate the pieces in this chunk
		//Place them and parent them
		this.chunkPieces = chunkPieces;
				
		
	}
	
	public void BindAllBlocksToMap(){
				
		for(int x = 0; x < width; x++){
		
			for(int y = 0; y < height; y++){
			
				int index = y * width + x;
				
				Block b = chunkPieces[index];
				
				b.BindToMap(this.x * parentMap.chunkWidth + x,this.y * parentMap.chunkHeight + y,depth,parentMap);
						
			}
			
		}
		
	}
	
	void InitializeBlockPlacement(int x, int y, Block b){
				
		if(b == null){
			return;
		}
		
		int index = y * width + x;
		
		Block eb = null;
		
		if(chunkPieces != null && index < chunkPieces.Length){
			eb = chunkPieces[index];
		}
		
		if(eb != null){
						
			Transform t = b.transform;
			t.parent = chunkTransform;
			t.localPosition = eb.gameObject.transform.localPosition;
						
			t.localRotation = eb.gameObject.transform.localRotation;
			
			
		}
		else{
			
			Transform t = b.transform;
									
			//position the piece
			t.parent = chunkTransform;
						
			float totalWidth, totalHeight;
			totalWidth = (float)width * parentMap.tileScale.x;
			totalHeight = (float)height * parentMap.tileScale.y;
			
			
			float xPos = -((float)x * parentMap.tileScale.x - ((float)totalWidth * 0.5f) + (0.5f * parentMap.tileScale.x));
			float yPos = -((float)y * parentMap.tileScale.y - ((float)totalHeight * 0.5f) + (0.5f * parentMap.tileScale.y));
	       
				
			//we'll need to invert this
			//to get top left to be 0,0
			//do that after testing
			Vector3 position = new Vector3(
			                               xPos,
			                               yPos,
			                               0.0f
			                               );
			
			
			t.localPosition = position;
			
			
			//t.localRotation = Quaternion.identity;
			t.up = parentMap.transform.up;
			t.forward = parentMap.transform.forward;
			
		}
		
		//b.BindToMap(this.x * parentMap.chunkWidth + x,this.y * parentMap.chunkHeight + y,depth,parentMap);
	}
	
	/// <summary>
	/// Has this chunk been initialized?
	/// </summary>
	/// <returns>
	///True if this chunk has been initialized, false if this chunk has not been initialized
	/// </returns>
	public bool Editor_IsInitialized(){
		//has this chunk been activated?
		//that is - has its chunk piece array been populated?
		return (chunkPieces != null && chunkPieces.Length > 0);
	}
	
	#region Expanding Map implementation
	/// <summary>
	/// Sets the coordinates of this chunk and its child blocks
	/// </summary>
	/// <param name="x">
	///The x coordinate to which to set this chunk.
	/// </param>
	/// <param name="y">
	///The y coordinate to which to set this chunk.
	/// </param>
	public void SetCoordinates (int x, int y)
	{
		this.name = "MapChunk_"+x+"_"+y;
		
		this.x = x;
		this.y = y;
		
		if(chunkPieces == null || chunkPieces.Length <= 0){
			return;
		}
		
		int abs_x = this.x * parentMap.chunkWidth;
		int abs_y = this.y * parentMap.chunkHeight;
		
		int w = parentMap.chunkWidth;
		int h = parentMap.chunkHeight;
		
		//and then set the x,y of the children
		for(int x1 = 0; x1 < w; x1++){
			for(int y1 = 0; y1 < h; y1++){
				
				int b_x, b_y;
				
				b_x = abs_x + x1;
				b_y = abs_y + y1;
				
				int index = y1 * w + x1;
				
				chunkPieces[index].x = b_x;
				chunkPieces[index].y = b_y;
				
			}			
		}
	}
	
	/// <summary>
	/// Get the Y coordinate of this chunk
	/// </summary>
	/// <returns>
	///The y coordinate of the chunk
	/// </returns>
	public int GetY(){
		return this.y;
	}
	
	/// <summary>
	/// Get the X coordinate of this chunk
	/// </summary>
	/// <returns>
	///The x coordinate of the chunk
	/// </returns>
	public int GetX(){
		return this.x;
	}
	
	#endregion
	
	/// <summary>
	///Returns the block map to which this chunk is bound
	/// </summary>
	/// <returns>
	///The blockmap to which this chunk is bound
	/// </returns>
	public BlockMap GetParentMap(){
		return parentMap;
	}
	
	#region Procedural Generation
	
	public void Proc_SetBlockAt(int x, int y, int z, Block b){
		
		
		
	}
		
	#endregion
		
	public void CleanChunk(){
		
		MapChunk m = this;
		
		//Delete empty chunks
		if(m.chunkPieces == null || m.chunkPieces.Length <= 0){
			
			//GameObject.Destroy(map[i].gameObject);
			
#if UNITY_4_0
			m.gameObject.SetActive(false);
#else
			m.gameObject.SetActiveRecursively(false);
#endif
			return;
		}
		
		if(m.chunkPieces != null){
			
			for(int j = 0; j < m.chunkPieces.Length; j++){
										
				if(m.chunkPieces[j] == null){
					continue;
				}
				
				if(m.chunkPieces[j].isNullBlock){
												
					//GameObject.Destroy(m.chunkPieces[j].gameObject);
					
					Renderer r = m.chunkPieces[j].GetComponent<Renderer>();
									
					if(r != null){
						GameObject.Destroy(r);
					}
					
					for(int i = 0; i < m.chunkPieces[j].gameObject.transform.childCount; i++){
						GameObject.Destroy(m.chunkPieces[j].gameObject.transform.GetChild(i).gameObject);
					}
					
					m.chunkPieces[j].GetComponent<Collider>().isTrigger = true;
					
					continue;
				}
				else if(!m.chunkPieces[j].retainCollider){
					
					m.chunkPieces[j].GetComponent<Collider>().isTrigger = true;
					
				}				
			}
			
		}
	}
}

