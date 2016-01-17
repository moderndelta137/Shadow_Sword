using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Utilities.Pathing;
using System.Collections.Generic;

namespace DopplerInteractive.TidyTileMapper.Utilities
{
	public class BlockUtilities
	{
		/// <summary>
		///Create a new Block Map  
		/// </summary>
		/// <param name="mapName">
		/// The name of your new block map
		/// </param>
		/// <param name="tileScale">
		/// The scale of the tiles within your new blockmap
		/// </param>
		/// <param name="chunkWidth">
		/// The width of the chunks within your new blockmap (Recommend: 5)
		/// </param>
		/// <param name="chunkHeight">
		/// The height of the chunks within your new blockmap (Recommend: 5)
		/// </param>
		/// <param name="growthAxis">
		/// The direction in which your new blockmap will grow when blocks are added to it
		/// </param>
		/// <returns>
		/// Your shiny, new blockmap
		/// </returns>
		public static BlockMap CreateBlockMap(string mapName, Vector3 tileScale, int chunkWidth, int chunkHeight, BlockMap.GrowthAxis growthAxis){
			
			GameObject o = new GameObject(mapName);
			
			BlockMap bm = o.AddComponent<BlockMap>();
			
			bm.tileScale = tileScale;
			bm.chunkWidth = chunkWidth;
			bm.chunkHeight = chunkHeight;
			bm.growthAxis = growthAxis;
						
			bm.editorMap = false;
			bm.hasBeenPublished = true;
			
			return bm;
			
		}
		
		static MapChunk GetChunkPrefab(){
			
			GameObject o = new GameObject("MapChunk");
			return o.AddComponent<MapChunk>();
			
		}
		
		/// <summary>
		///Returns an empty block (useful for deleting blocks but retaining trigger functionality) 
		/// </summary>
		/// <param name="map">
		/// The map to which this block will be added (required in order to correctly size the trigger of the block)
		/// </param>
		/// <returns>
		/// Returns an empty block
		/// </returns>
		public static Block GetEmptyBlock(BlockMap map){
			
			GameObject o = null;
			
			if(AssetPool.IsPoolingEnabled()){
				
				o = AssetPool.Instantiate("Void");
				
				if(o != null){
					return o.GetComponent<Block>();
				}
				
			}
			
			o = new GameObject("Void");
			OrientedBlock ob = o.AddComponent<OrientedBlock>();
			BoxCollider b = o.AddComponent<BoxCollider>();
			b.isTrigger = true;
			b.size = map.tileScale;
			ob.isNullBlock = true;
			ob.actAsEmptyBlock = true;
			return ob as Block;
		
			
		}
				
		/// <summary>
		///Remove a block from the map (replacing it with an Empty Block) 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to remove the block
		/// </param>
		/// <param name="x">
		/// The x coordinate of the block
		/// </param>
		/// <param name="y">
		/// The y coordinate of the block
		/// </param>
		/// <param name="depth">
		/// The depth of the block
		/// </param>
		/// <param name="addEmptyBlock">
		/// Add an empty block to the void left after removal?
		/// </param>
		/// <param name="destroyExistingImmediate">
		/// Use DestroyImmediate or Destroy when removing the existing block? (You must use immediate from within the Editor, and cannot use it once physics collisions)
		/// </param>
		public static void RemoveBlockFromMap(BlockMap map, int x, int y, int depth, bool addEmptyBlock, bool destroyImmediate){
			
			MapChunk m = map.GetChunkForBlockCoordinate(x,y,depth);
			
			if(m != null){
				
				Block eb = map.GetBlockAt(x,y,depth);
			
				if(eb != null){
					
					map.SetBlockAt(x,y,depth,null,destroyImmediate);
					
				}
				
				if(addEmptyBlock){
					map.SetBlockAt(x,y,depth,GetEmptyBlock(map),destroyImmediate);
				}
				
			}
			
		}
				
		/// <summary>
		///Set the variant of the targetted block 
		/// </summary>
		/// <param name="map">
		///The blockmap containing the target block
		/// </param>
		/// <param name="x">
		///X coordinate of the target block
		/// </param>
		/// <param name="y">
		///Y coordinate of the target block
		/// </param>
		/// <param name="depth">
		///Z (depth) coordinte of the target block
		/// </param>
		/// <param name="variant">
		///Index of the variant to which to set this block
		/// </param>
		public static void SetBlockVariant(BlockMap map, int x, int y, int depth, int variant){
			
			MapChunk m = map.GetChunkForBlockCoordinate(x,y,depth);
			
			if(m != null){
				
				Block eb = map.GetBlockAt(x,y,depth);
			
				if(eb != null){
					
					eb.SetVariant(variant);
					RefreshBlock(map,x,y,depth);					
				}
				
			}
			
		}
		
		/// <summary>
		///Refresh the map - correcting orientations on all blocks 
		/// </summary>
		/// <param name="map">
		/// The map that you wish to refresh
		/// </param>
		/// <param name="randomise">
		/// Randomise the block variations within the map?
		/// </param>
		public static void RefreshMap(BlockMap map, bool randomise){
			map.RefreshMap();
			
			if(randomise){
				for(int x = 0; x < map.GetMapWidth(); x++){
					for(int y = 0; y < map.GetMapHeight(); y++){
						for(int z = map.mapLowerDepth; z <= map.mapUpperDepth; z++){
							Block b = map.GetBlockAt(x,y,z);
							if(b != null && !b.isNullBlock){
								b.RandomiseVariant();
							}
						}
					}
				}
			}
		}
		
		/// <summary>
		/// Cleans the blockmap - removing all unused variations and orientations 
		/// </summary>
		/// <param name="map">
		/// The map that you wish to clean
		/// </param>
		public static void CleanMap(BlockMap map){
			map.CleanMap();
		}
		
		/// <summary>
		///Get the block located at the given coordinates 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve a block
		/// </param>
		/// <param name="x">
		/// The x coordinate of the block
		/// </param>
		/// <param name="y">
		/// The y coordinate of the block
		/// </param>
		/// <param name="depth">
		/// The depth of the block
		/// </param>
		/// <returns>
		/// The block, if one exists, null if not.
		/// </returns>
		public static Block GetBlockAt(BlockMap map, int x, int y, int depth){
			return map.GetBlockAt(x,y,depth);
		}
		
		/// <summary>
		///Returns the lowest depth that is populated within the given map 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve the depth
		/// </param>
		/// <returns>
		/// The lowest depth recorded in the map
		/// </returns>
		public static int GetMapLowerDepth(BlockMap map){
			return map.mapLowerDepth;
		}
		
		/// <summary>
		///Returns the highest depth that is populated within the given map 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve the depth
		/// </param>
		/// <returns>
		/// The highest depth recorded in the map
		/// </returns>
		public static int GetMapUpperDepth(BlockMap map){
			return map.mapUpperDepth;
		}
		
		/// <summary>
		///Returns the height (in tiles) of the map 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve the height
		/// </param>
		/// <returns>
		/// The height of the map
		/// </returns>
		public static int GetMapHeight(BlockMap map){
			return map.GetMapHeight();
		}
		
		/// <summary>
		///Returns the width (in tiles) of the map 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve the width
		/// </param>
		/// <returns>
		/// The width of the map
		/// </returns>
		public static int GetMapWidth(BlockMap map){
			return map.GetMapWidth();
		}
		
		/// <summary>
		///Add a block to the desired map 
		/// </summary>
		/// <param name="map">
		/// The map to which you wish to add a block
		/// </param>
		/// <param name="b">
		/// The instantiated Block object that you wish to add to the map (if null, will add an EmptyBlock object)
		/// </param>
		/// <param name="randomise">
		/// Randomise the variant of this Block upon addition to the map?
		/// </param>
		/// <param name="variation">
		/// If not randomising, feel free to pass through the index of the variation you wish to display
		/// </param>
		/// <param name="refreshUponAddition">
		/// Refresh the surrounding blocks upon addition to the map?
		/// </param>
		/// <param name="x">
		/// The target x coordinate
		/// </param>
		/// <param name="y">
		/// The target y coordinate
		/// </param>
		/// <param name="depth">
		/// The depth to which to add the block
		/// </param>
		/// <param name="destroyExistingImmediate">
		/// Use DestroyImmediate or Destroy when removing the existing block? (You must use immediate from within the Editor, and cannot use it once physics collisions)
		/// </param>
		/// <param name="addEmptyWhenNull">
		/// When passing 'null', should we add an empty block, or should we simply destroy the space and leave it?
		/// </param>
		public static void AddBlockToMap(BlockMap map, Block b, bool randomise, int variation, bool refreshUponAddition, int x, int y, int depth, bool destroyExistingImmediate, bool addEmptyWhenNull){
			
			if(b == null && addEmptyWhenNull){
				b = GetEmptyBlock(map);
			}
			
			MapChunk m = map.GetChunkForBlockCoordinate(x,y,depth);
			
			if(m == null){
								
				int mx = 0; 
				
				mx = (int)((float)x / (float)map.chunkWidth);
				
				int my = 0; 
				
				my = (int)((float)y / (float)map.chunkHeight);
				
				MapChunk mc = GetChunkPrefab();
				
				map.Editor_AddChunkAt(mx,my,depth,mc,true);
				
				if(destroyExistingImmediate){
					GameObject.DestroyImmediate(mc.gameObject);
				}
				else{
					GameObject.Destroy(mc.gameObject);
				}
				
				m = map.GetChunkForBlockCoordinate(x,y,depth);
				
				Block[] cb = new Block[map.chunkWidth * map.chunkHeight];
				
				m.Editor_InitializeChunk(cb);
				
			}
			
			//Block eb = map.GetBlockAt(x,y,depth);
			
			map.SetBlockAt(x,y,depth,null,destroyExistingImmediate);		
			
			map.SetBlockAt(x,y,depth,b,destroyExistingImmediate);
			
			if(randomise && b != null){
				b.RandomiseVariant();
			}
			else if(b != null && variation != 0){
				b.SetVariant(variation);
			}
			
			if(refreshUponAddition){
				RefreshBlock(map,x,y,depth);
			}
			
			if(b != null && !b.retainCollider){
				Collider c = b.GetComponent<Collider>();
				
				if(c != null){
					
					if(addEmptyWhenNull){
						c.isTrigger = true;
					}
					else{
						if(destroyExistingImmediate){
							GameObject.DestroyImmediate(c);
						}
						else{
							GameObject.Destroy(c);
						}
					}
				}
				
			}
			
		}
		
		/// <summary>
		/// Returns a path of tile coordinates from the source x,y coords to target x,y coords, at the given depth.
		/// </summary>
		/// <param name="map">
		/// The BlockMap over which to find a path
		/// </param>
		/// <param name="from_x">
		/// The X map coordinate from which to path
		/// </param>
		/// <param name="from_y">
		/// The Y map coordinate from which to path
		/// </param>
		/// <param name="to_x">
		/// The X map coordinate to which to path
		/// </param>
		/// <param name="to_y">
		/// The Y map coordinate to which to path
		/// </param>
		/// <param name="depth">
		/// The map depth at which to path
		/// </param>
		/// <param name="useDiagonals">
		/// Use diagonals when pathing? (Otherwise we will only use 90 degree turns)
		/// </param>
		/// <returns>
		/// Returns a list of tile coordinates, empty if no path could be found
		/// </returns>
		public static List<Vector3> GetPath(BlockMap map, int from_x, int from_y, int to_x, int to_y, int depth, bool useDiagonals){
			
			List<PathNode> path = PathFinding.GetPath(from_x,from_y,to_x,to_y,map,depth,useDiagonals,false,0,false);
			
			List<Vector3> coords = new List<Vector3>();
			
			if(path == null || path.Count <= 0){
				return coords;
			}
			
			for(int i = 0; i < path.Count; i++){
								
				coords.Add(new Vector3(path[i].x,path[i].y,depth));
			}
						
			return coords;
						
		}
		
		/// <summary>
		/// Is this point within the bounds of the map? 
		/// </summary>
		/// <param name="map">
		/// The map we wish to query
		/// </param>
		/// <param name="x">
		/// The test X coordinate
		/// </param>
		/// <param name="y">
		/// The test Y coordinate
		/// </param>
		/// <param name="z">
		/// The test z coordinate
		/// </param>
		/// <returns>
		/// True if the point lies within the bounds of the map, false if otherwise.
		/// </returns>
		public static bool IsWithinMapBounds(BlockMap map, int x, int y, int z){
			
			int lowerXBound = 0;
			int lowerYBound = 0;
			
			int upperXBound = GetMapWidth(map);
			int upperYBound = GetMapHeight(map);
			
			if(map.editorMap){
				
				lowerXBound = map.chunkWidth;
				lowerYBound = map.chunkHeight;
				upperXBound = upperXBound - map.chunkWidth;
				upperYBound = upperYBound - map.chunkHeight;
				
			}
			
			if(x < lowerXBound || (x >= upperXBound)){
				return false;
			}
			
			if(y < lowerYBound || (y >= upperYBound)){
				return false;
			}
			
			if(z < GetMapLowerDepth(map) || z > GetMapUpperDepth(map)){
				return false;
			}
			
			return true;
			
		}
		
		/// <summary>
		/// Refresh the target block - correcting the orientation of the block and the surrounding blocks 
		/// </summary>
		/// <param name="map">
		/// The map housing the block that you wish to refresh
		/// </param>
		/// <param name="x">
		/// The x coordinate of the block to be refreshed
		/// </param>
		/// <param name="y">
		/// The y coordinate of the block to be refreshed
		/// </param>
		/// <param name="depth">
		/// The depth of the block to be refreshed
		/// </param>
		public static void RefreshBlock(BlockMap map, int x, int y, int depth){
						
			for(int x1 = x-1; x1 <= x+1; x1++){
				for(int y1 = y-1; y1 <= y+1; y1++){
					
					Block b = map.GetBlockAt(x1,y1,depth);
					
					if(b != null){
						b.RefreshBlock();
					}
					
				}
			}
			
		}
		
		/// <summary>
		/// Cleans a single block 
		/// </summary>
		/// <param name="map">
		/// The map containing the target block
		/// </param>
		/// <param name="x">
		/// The x coordinate of the target block
		/// </param>
		/// <param name="y">
		/// The y coordinate of the target block
		/// </param>
		/// <param name="depth">
		/// The depth coordinate of the target block
		/// </param>
		public static void CleanBlock(BlockMap map, int x, int y, int depth){
			
			Block b = map.GetBlockAt(x,y,depth);
					
			if(b != null){
				
				OrientedBlock ob = (OrientedBlock) b;
				
				if(ob == null){
					return;
				}
				
				BlockSet[] bs = ob.GetBlockSetsAsArray();
				
				BlockSet currentSet = ob.GetCurrentBlockSet();
				
				for(int k = 0; k < bs.Length; k++){
					
					if(bs[k] == currentSet){
						continue;
					}
					
					BlockSet bSet = bs[k];
					
					for(int l = 0; l < bSet.blockSet.Length; l++){
						
						GameObject.Destroy(bSet.blockSet[l].gameObject);
						
					}
				}
				
					
			}
			
		}
		
		/// <summary>
		/// Returns the chunk at the given block coordinates 
		/// </summary>
		/// <param name="map">
		/// The map from which to retrieve the chunk
		/// </param>
		/// <param name="x">
		/// The block x coordinate
		/// </param>
		/// <param name="y">
		/// The block y coordinate
		/// </param>
		/// <param name="depth">
		/// The depth coordinate
		/// </param>
		/// <returns>
		/// The chunk (if one exists) - null if not
		/// </returns>
		public static MapChunk GetChunkAtCoordinates(BlockMap map, int x, int y, int depth){
			
			return map.GetChunkForBlockCoordinate(x,y,depth);
			
		}
		
		/// <summary>
		/// Returns the local position of the given coordinate (useful for character movement) 
		/// </summary>
		/// <param name="map">
		/// The map from which you wish to retrieve the position
		/// </param>
		/// <param name="x">
		/// The block x coordinate
		/// </param>
		/// <param name="y">
		/// The block y coordinate
		/// </param>
		/// <param name="depth">
		/// The block depth coordinate
		/// </param>
		/// <returns>
		/// The local position that is central to this block coordinate
		/// </returns>
		public static Vector3 GetMathematicalPosition(BlockMap map, int x, int y, int depth){
			
			Vector3 position = Vector3.zero;
						
			float halfChunk_x = (float)(map.chunkWidth) * map.tileScale.x * 0.5f;
			
			if(map.editorMap){
				MapChunk c = map.GetLeftMostMapChunk();
				if(c != null){
					halfChunk_x += c.transform.localPosition.x;
				}
				
			}
			
			float halfChunk_y = 0.0f;
			
			//Vector3 upperLeft = Vector3.zero;
			
			if(map.growthAxis == BlockMap.GrowthAxis.Up){
				halfChunk_y = (float)(map.chunkHeight) * map.tileScale.y * 0.5f;
				
				MapChunk c = map.GetTopMostMapChunk();
				if(c != null){
					
					halfChunk_y += c.transform.localPosition.y;
					
				}
				
				position.y = -(y * map.tileScale.y + (0.5f * map.tileScale.y) - halfChunk_y);
				position.z = depth * map.tileScale.z;
			}
			else{
				halfChunk_y = (float)(map.chunkHeight) * map.tileScale.z * 0.5f;
				
				MapChunk c = map.GetTopMostMapChunk();
				if(c != null){
					halfChunk_y += c.transform.localPosition.z;
				}
				
				position.z = -(y * map.tileScale.z + (0.5f * map.tileScale.z) - halfChunk_y);
				position.y = depth * map.tileScale.y;
			}
			
			position.x = -((x * map.tileScale.x) + (0.5f * map.tileScale.x) - halfChunk_x);
			
			
			return position;
			
		}
		
		/// <summary>
		/// Mathematically divine the coordinates of the global position of an object within a map 
		/// </summary>
		/// <param name="map">
		/// The map within which you wish to divine your coordinates
		/// </param>
		/// <param name="globalPosition">
		/// The global position of the transform for which to divine coordinates
		/// </param>
		/// <returns>
		/// The x,y,z coordinates of the transform, gathered mathematically. NOTE: This is not of use on a map that involves arbitrarily-placed blocks (e.g soft selection)
		/// </returns>
		public static Vector3 GetMathematicalCoordinates(BlockMap map, Vector3 globalPosition){
			
			Vector3 localPosition = map.transform.InverseTransformPoint(globalPosition);
			
			int x = 0;
			int y = 0;
			int z = 0;
			
			float halfChunk_x = (float)(map.chunkWidth) * map.tileScale.x * 0.5f;
						
			if(map.editorMap){
				MapChunk c = map.GetLeftMostMapChunk();
				if(c != null){
					halfChunk_x += c.transform.localPosition.x;
				}
				
			}
			
			float halfChunk_y = 0.0f;
			
			if(map.growthAxis == BlockMap.GrowthAxis.Up){
				halfChunk_y = (float)(map.chunkHeight) * map.tileScale.y * 0.5f;
				
				MapChunk c = map.GetTopMostMapChunk();
				if(c != null){
					halfChunk_y += c.transform.localPosition.y;
				}
			}
			else{
				halfChunk_y = (float)(map.chunkHeight) * map.tileScale.z * 0.5f;
				
				MapChunk c = map.GetTopMostMapChunk();
				if(c != null){
					halfChunk_y += c.transform.localPosition.z;
				}
			}
			
			Vector3 modPos = localPosition;
									
			modPos.x = (localPosition.x - halfChunk_x - (0.5f * map.tileScale.x));
			
			if(map.growthAxis == BlockMap.GrowthAxis.Up){
				modPos.y = (localPosition.y - halfChunk_y + (0.5f * map.tileScale.y));
			}
			else{
				modPos.z = (localPosition.z - halfChunk_y + (0.5f * map.tileScale.z));
			}
			
			x = (int)((modPos.x- 0.5f * map.tileScale.x) / map.tileScale.x) + 1;
			
			if(map.growthAxis == BlockMap.GrowthAxis.Up){
				y = (int)((modPos.y - 0.5f * map.tileScale.y) / map.tileScale.y);
			}
			else{
				y = (int)((modPos.z - 0.5f * map.tileScale.z) / map.tileScale.z);
			}
			
			if(map.growthAxis == BlockMap.GrowthAxis.Up){
				modPos.z += (-map.mapLowerDepth * map.tileScale.z);
				z = (int)(((modPos.z + 0.5f * map.tileScale.z) / map.tileScale.z) + map.mapLowerDepth);
			}
			else{
				modPos.z += (-map.mapLowerDepth * map.tileScale.y);
				z = (int)(((modPos.y - 0.5f * map.tileScale.y) / map.tileScale.y) + map.mapLowerDepth);
			}
						
			return new Vector3(-x,-y,z);
		}
		
		public static char MAP_STRING_FIELD_DELIM = '^';
		public static char BLOCK_FIELD_DELIM = '%';
		public static char MAP_ENTITY_DELIM = '*';
		
		/// <summary>
		/// Creates a Blockmap from the provided save string. I bet you'd like to know how you can use this! :D No dice.
		/// </summary>
		/// <param name="blockMapString">
		/// The save string.
		/// </param>
		/// <returns>
		/// The created BlockMap, if creation was successful.
		/// </returns>
		public static BlockMap StringToBlockMap(string blockMapString){
			
			BlockMap map = null;
			
			string[] mapFields = blockMapString.Split(MAP_STRING_FIELD_DELIM);
			
			string mapName = mapFields[0];
			
			string[] scale = mapFields[1].Split(',');
			
			float tileScale_x,tileScale_y,tileScale_z;
			
			tileScale_x = float.Parse(scale[0]);
			tileScale_y = float.Parse(scale[1]);
			tileScale_z = float.Parse(scale[2]);
			
			Vector3 tileScale = new Vector3(tileScale_x,tileScale_y,tileScale_z);
			
			Debug.Log(mapFields[2]);
			Debug.Log(mapFields[3]);
			
			int chunkWidth = int.Parse(mapFields[2]);
			int chunkHeight = int.Parse(mapFields[3]);
			
			int growthAxis_int = int.Parse(mapFields[4]);
			
			BlockMap.GrowthAxis growthAxis = (BlockMap.GrowthAxis)growthAxis_int;
			
			int width = int.Parse(mapFields[5]);
			int height = int.Parse(mapFields[6]);
			int lowerDepth = int.Parse(mapFields[7]);
			int upperDepth = int.Parse(mapFields[8]);
			
			string[] blockLib = mapFields[9].Split(BLOCK_FIELD_DELIM);
			
			Debug.Log("Map name: " + mapName);
			Debug.Log("Tile Scale: " + tileScale.ToString());
			Debug.Log("Chunk width: " + chunkWidth + " Chunk height: " + chunkHeight);
			Debug.Log("Growth axis: " + growthAxis.ToString());
			Debug.Log("Lower depth: " + lowerDepth + " Upper depth: " + upperDepth);
			Debug.Log("Width: " + width + " Height: " + height);
			
			string libString = "";
			
			for(int i = 0; i < blockLib.Length; i++){
				libString += blockLib[i] + " ";
			}
			
			Debug.Log("Block library: " + libString);
			
			Debug.Log("All blocks:");
			
			for(int i = 10; i < mapFields.Length; i++){
				
				if(mapFields[i] == ""){
					Debug.Log("Empty block");
				}
				else{
					StringToBlock(mapFields[i]);
				}
				
			}
			
			return map;
			
		}
		
		/// <summary>
		/// Creates an individual Block from the provided save string. 
		/// </summary>
		/// <param name="blockString">
		/// The save string for this block.
		/// </param>
		/// <returns>
		/// The Block, if creation was successful.
		/// </returns>
		public static Block StringToBlock(string blockString){
			
			string[] blockFields = blockString.Split(BLOCK_FIELD_DELIM);
			
			string index_str = blockFields[0];
			string variant_str = blockFields[1];
			
			int variant = int.Parse(variant_str);
			int index = int.Parse(index_str);
			
			string blockCoords_str = blockFields[2];
			
			string[] cSplit = blockCoords_str.Split(',');
			
			int x = int.Parse(cSplit[0]);
			int y = int.Parse(cSplit[1]);
			int z = int.Parse(cSplit[2]);
			
			Debug.Log("Block index: " + index + " Block variant: " + variant + " Pos: " + x + ","+ y+","+z);
				
			for(int i = 3; i < blockFields.Length; i++){
				
				Debug.Log("Block has subscriber: " + blockFields[i]);
				
			}
			
			return null;
			
		}
		
		/// <summary>
		/// Create a save string from the provided BlockMap 
		/// </summary>
		/// <param name="map">
		/// The BlockMap from which to create a save string.
		/// </param>
		/// <returns>
		/// The save string representing this BlockMap.
		/// </returns>
		public static string BlockmapToString(BlockMap map){
			
			string mapString = "";
			
			//Add the map name
			mapString += map.name + MAP_STRING_FIELD_DELIM;
			//Add the tile scale			
			mapString += map.tileScale.x + "," + map.tileScale.y + "," + map.tileScale.z + MAP_STRING_FIELD_DELIM;
			//Add the chunk width
			mapString += map.chunkWidth + ""+ MAP_STRING_FIELD_DELIM;
			//Add the chunk height
			mapString += map.chunkHeight + ""+ MAP_STRING_FIELD_DELIM;
			//Add the growth axis
			mapString += ((int)map.growthAxis) + ""+ MAP_STRING_FIELD_DELIM;
						
			int lowerDepth = BlockUtilities.GetMapLowerDepth(map);
			int upperDepth = BlockUtilities.GetMapUpperDepth(map);
			int width = BlockUtilities.GetMapWidth(map);
			int height = BlockUtilities.GetMapHeight(map);
			
			//Now add the map height and width and upper and lower depth
			mapString += width + ""+ MAP_STRING_FIELD_DELIM + ""+ height + ""+MAP_STRING_FIELD_DELIM;
			mapString += lowerDepth + ""+ MAP_STRING_FIELD_DELIM + ""+ upperDepth + ""+ MAP_STRING_FIELD_DELIM;
			
			//Now we need to populate our block library
			List<string> blockLibrary = new List<string>();
						
			for(int z = lowerDepth; z <= upperDepth; z++){
				
				for(int x = 0; x < width; x++){
					
					for(int y = 0; y < height; y++){
					
						Block b = BlockUtilities.GetBlockAt(map,x,y,z);
						
						if(b != null && !b.isNullBlock){
							
							if(!blockLibrary.Contains(b.name)){
								blockLibrary.Add(b.name);
							}
							
						}
					}
					
				}				
			}
			
			//Debug only
			string blockStr = "Populated block library: ";
			
			for(int i = 0; i < blockLibrary.Count; i++){
				
				//Debug only
				blockStr += blockLibrary[i] + ",";
				
				mapString += blockLibrary[i] + BLOCK_FIELD_DELIM;
			}
			
			mapString += MAP_STRING_FIELD_DELIM;
						
			Debug.Log(blockStr);
			
			for(int z = lowerDepth; z <= upperDepth; z++){
				
				for(int x = 0; x < width; x++){
					
					for(int y = 0; y < height; y++){
					
						Block b = BlockUtilities.GetBlockAt(map,x,y,z);
						
						if(b != null && !b.isNullBlock){
							
							mapString += BlockUtilities.BlockToString(b,blockLibrary);
														
						}
						else{
							
							mapString += "-1" + BLOCK_FIELD_DELIM + "0" + BLOCK_FIELD_DELIM +""+ x +"," +y+","+z+""+BLOCK_FIELD_DELIM;
							
						}
						
						mapString += MAP_STRING_FIELD_DELIM;
					}
					
				}				
			}
						
			return mapString;
			
		}
		
		static int GetIndexForBlock(string blockName, List<string> library){
			
			for(int i = 0; i < library.Count; i++){
				
				if(library[i] == blockName){
					return i;
				}
				
			}
			
			Debug.LogWarning("No block found in library for: " + blockName + " when saving map as string.");
			
			return -1;
			
		}
		
		/// <summary>
		/// Creates a save string for an individual Block 
		/// </summary>
		/// <param name="b">
		/// The Block in question
		/// </param>
		/// <param name="library">
		/// The type library constructed in the body of the save Map function
		/// </param>
		/// <returns>
		/// The save string for this block.
		/// </returns>
		public static string BlockToString(Block b,List<string> library){
			
			OrientedBlock ob = b as OrientedBlock;
			
			string blockString = "";
			
			blockString += GetIndexForBlock(ob.name,library) + ""+ BLOCK_FIELD_DELIM;
						
			blockString += ob.GetCurrentVariant() + ""+ BLOCK_FIELD_DELIM;
			
			blockString += ob.x + "," + ob.y + "," + ob.depth + ""+ BLOCK_FIELD_DELIM;
			
			TidyMapBoundObject[] boundObjects = ob.GetComponentsInChildren<TidyMapBoundObject>();
			
			for(int i = 0; i < boundObjects.Length; i++){
				
				blockString += boundObjects[i].ToString() + ""+ BLOCK_FIELD_DELIM;
				
			}
			
			return blockString;
			
		}
	}
}

