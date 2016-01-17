using System;
using UnityEngine;
using System.Collections.Generic;

namespace DopplerInteractive.TidyTileMapper.Layering
{
	/// <summary>
	///A chunkset is a collection of chunks, used for layering 
	/// </summary>
	[Serializable]
	public class ChunkSet
	{
		[Serializable]
		public class MapChunkEntry{
			public MapChunk chunk;
			public int depth;
			
			public MapChunkEntry(MapChunk chunk, int depth){
				this.chunk = chunk;
				this.depth = depth;
			}
		};
		
		public int x, y;
		
		public BlockMap parentMap;
		
		
		public MapChunkEntry[] chunkSet;
		
		public void RemoveChunkAt(int depth){
			
			for(int i = 0; i < chunkSet.Length; i++){
				if(chunkSet[i] != null){
					if(chunkSet[i].depth == depth){
						chunkSet[i] = null;
					}
				}
			}
			
			CleanChunkSet();
			
		}
		
		void CleanChunkSet(){
			
			List<MapChunkEntry> cleanSet = new List<MapChunkEntry>();
			
			for(int i = 0; i < chunkSet.Length; i++){
				if(chunkSet[i] != null){
					cleanSet.Add(chunkSet[i]);
				}
			}
			
			chunkSet = cleanSet.ToArray();
			
		}
		
		public void InitializeChunkSet(BlockMap parentMap, int x, int y){
			this.x = x;
			this.y = y;
			this.parentMap = parentMap;
		}
		
		/// <summary>
		///Returns the chunk at the given depth 
		/// </summary>
		/// <param name="depth">
		///The depth of the chunk: Note this is relative to the original map height, so this may be negative
		/// </param>
		/// <returns>
		/// <param name="returnClosest">
		///Should this return the closest depth?
		/// </param>
		/// <returns>
		///Returns the MapChunk.
		/// </returns>
		public MapChunk GetChunkAt(int depth, bool returnClosest){
			if(chunkSet == null){
				return null;
			}
			
			if(returnClosest){
				if(depth > GetUpperChunkSetBound()){
					depth = GetUpperChunkSetBound();
				}
				else
				if(depth < GetLowerChunkSetBound()){
					depth = GetLowerChunkSetBound();
				}
			}
			
			for(int i =0 ; i < chunkSet.Length; i++){
				if(chunkSet[i].depth == depth){
					return chunkSet[i].chunk;
				}
			}
			
			return null;
			
		}
		
		/// <summary>
		///Returns the lowest bound of the chunkset 
		/// </summary>
		/// <returns>
		///The lowest level of the chunkset - may be negative
		/// </returns>
		public int GetLowerChunkSetBound(){
			if(chunkSet == null){
				return 0;
			}
			
			int lowestDepth = 0;
			
			for(int i = 0; i < chunkSet.Length; i++){
				if(chunkSet[i].depth < lowestDepth){
					lowestDepth = chunkSet[i].depth;
				}
			}
			
			return lowestDepth;
			
		}
		
		/// <summary>
		///Returns the highest bound of the chunkset 
		/// </summary>
		/// <returns>
		///The highest level of the chunkset - greater than or equal to 0
		/// </returns>
		public int GetUpperChunkSetBound(){
			if(chunkSet == null){
				return 0;
			}
			
			int highestDepth = 0;
			
			for(int i = 0; i < chunkSet.Length; i++){
				if(chunkSet[i].depth > highestDepth){
					highestDepth = chunkSet[i].depth;
				}
			}
			
			return highestDepth;
		}
		
		/// <summary>
		///Add a chunk to the chunkset
		/// </summary>
		/// <param name="chunk">
		///The chunk you wish to add
		/// </param>
		/// <param name="depth">
		///The depth at which you are adding the chunk
		/// </param>
		public void AddChunkAt(MapChunk chunk, int depth){
			
			if(chunkSet == null || chunkSet.Length == 0){
				chunkSet = new MapChunkEntry[1];
				chunkSet[0] = new ChunkSet.MapChunkEntry(chunk,depth);
				return;
			}
			
			if(ContainsDepth(depth)){
				
				Debug.LogWarning("Attemping to add a chunk at depth: " + depth + " to set at " + x + "," + y);
				
				return;
			}
			
			int newLength = chunkSet.Length + 1;
			
			MapChunkEntry[] newSet = new MapChunkEntry[newLength];
			
			for(int i = 0; i < chunkSet.Length; i++){
				
				newSet[i] = chunkSet[i];
				
			}
			
			newSet[newLength-1] =  new ChunkSet.MapChunkEntry(chunk,depth);
			
			chunkSet = newSet;
			
		}
		
		/// <summary>
		///Does this chunkset contain a chunk at this depth? 
		/// </summary>
		/// <param name="depth">
		///The depth you are querying
		/// </param>
		/// <returns>
		///True if chunkset contains this depth, false if otherwise
		/// </returns>
		bool ContainsDepth(int depth){
			
			for(int i =0 ; i < chunkSet.Length; i++){
				if(chunkSet[i].depth == depth){
					return true;
				}
			}
			
			return false;
			
		}
		
		/// <summary>
		///Does this chunkset contain any chunks? 
		/// </summary>
		/// <returns>
		///True if this set contains chunks, otherwise false
		/// </returns>
		public bool HasChunks(){
			if(chunkSet == null || chunkSet.Length == 0){
				return false;
			}
			
			return true;
			
		}	
		
		public void RefreshChunkSet(){
			
			if(chunkSet == null){
				return;
			}
			
			for(int i = 0; i < chunkSet.Length; i++){
				
				if(chunkSet[i] == null){
					continue;
				}
				
				if(chunkSet[i].chunk != null){
					chunkSet[i].chunk.RefreshChunk();
				}
			}
			
		}
				
		#region Expanding Map logic
		
		/// <summary>
		///Set the coordinates of this chunkset 
		/// </summary>
		/// <param name="x">
		///The new x coordinate of this chunkset
		/// </param>
		/// <param name="y">
		///The new y coordinate of this chunkset
		/// </param>
		public void SetCoordinates (int x, int y)
		{
			this.x = x;
			this.y = y;
		
			if(chunkSet == null){
				return;
			}
			
			for(int i = 0; i < chunkSet.Length; i++){
				
				chunkSet[i].chunk.SetCoordinates(x,y);
			
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
				
		/// <summary>
		///Returns the block map to which this chunk is bound
		/// </summary>
		/// <returns>
		///The blockmap to which this chunk is bound
		/// </returns>
		public BlockMap GetParentMap(){
			return parentMap;
		}
		
		#endregion
	}
}

