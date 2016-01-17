using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Layering;
using DopplerInteractive.TidyTileMapper.Utilities.Pathing;
using DopplerInteractive.TidyTileMapper.Utilities;
using System.Collections.Generic;

public class BlockMap : ExpandingMap, IPathMap
{
	
	//Rebuild please
	
	/// <summary>
	///The width of the chunks contained within the map, measured in blocks
	/// </summary>
	public int chunkWidth;
	/// <summary>
	///The height of the chunks contained within the map, measured in blocks
	/// </summary>
	public int chunkHeight;
			
	/// <summary>
	///The scale of the blocks contained within the map, in Unity scale 
	/// </summary>
	public Vector3 tileScale;
		
	/// <summary>
	///Is this an editor map, or has it been published? 
	/// </summary>
	public bool editorMap = true;
	public bool hasBeenPublished = false;
	
	/// <summary>
	///The axis that a map may grow: Upward or outward 
	/// </summary>
	public enum GrowthAxis{
		Up,
		Forward
	}
	
	/// <summary>
	///The axis upon which this map will grow 
	/// </summary>
	public GrowthAxis growthAxis;
	
	#region Layering
	
	public int mapUpperDepth;
	public int mapLowerDepth;
	
	#endregion
	
	#region Functional Overlays
	
	public TidyFunctionalOverlay functionalOverlay = null;
	
	#endregion
	
	#region Background mesh
	
	public List<BackgroundEntry> backgrounds = new List<BackgroundEntry>();
	public Dictionary<Vector3, BackgroundEntry> worldCoordMappings = new Dictionary<Vector3, BackgroundEntry>();
	
	void RefreshCoordMappings(){
		
		worldCoordMappings.Clear();
		
		for(int i = 0; i < backgrounds.Count; i++){
			
			BackgroundEntry b = backgrounds[i];
			
			Vector3[] elements = b.GetElementsArray();
			
			for(int j = 0; j < elements.Length; j++){
				
				worldCoordMappings.Add(elements[j],b);
				
			}
			
		}
		
	}
	
	void InitializeBackground(){
		
		RefreshCoordMappings();
		
		
		
		for(int i = 0; i < backgrounds.Count; i++){
			
			backgrounds[i].Initialize();
			
		}
		
	}
	
	[Serializable]
	public class BackgroundEntry{
		
		public GameObject backgroundObject;
		public Material backgroundMaterial;
		public Renderer backgroundRenderer;
		public MeshFilter backgroundMeshFilter;
		
		[HideInInspector]
		public List<Vector3> backgroundElements = new List<Vector3>();
		[HideInInspector]
		public HashSet<Vector3> elementSet = null;
		public Dictionary<Vector3,int> vDictionary = new Dictionary<Vector3, int>();
			
		public void Initialize(){
					
			if(elementSet != null){
				return;
			}
						
			elementSet = new HashSet<Vector3>();
			
			for(int i = 0; i < backgroundElements.Count; i++){
				elementSet.Add(backgroundElements[i]);
			}
			
			backgroundElements.Clear();
			
		}
		
		public void AddCoordinate(Vector3 coord){
			
			if(Application.isPlaying){
				
				if(elementSet == null){
					Initialize();
				}
				elementSet.Add(coord);
				
			}
				
			backgroundElements.Add(coord);
				
			
		}
		
		public Vector3[] GetElementsArray(){
			
			Vector3[] elements = null;
			
			elements = backgroundElements.ToArray();
									
			return elements;
			
		}
				
		public bool ContainsWorldCoord(Vector3 worldCoord){
			
			if(elementSet != null){
				return elementSet.Contains(worldCoord);
			}
			else if(Application.isPlaying){
				Initialize();
				return ContainsWorldCoord(worldCoord);
			}
			
			for(int i = 0; i < backgroundElements.Count; i++){
				if(backgroundElements[i] == worldCoord){
					return true;
				}
			}
			
			return false;
			
		}
		
		public void RemoveEntryFor(Vector3 worldCoord){
			
			if(elementSet != null){
				elementSet.Remove(worldCoord);
			}
			else if(Application.isPlaying){
				Initialize();
				elementSet.Remove(worldCoord);
			}
			
			for(int i = 0; i < backgroundElements.Count; i++){
				if(backgroundElements[i] == worldCoord){
					backgroundElements.RemoveAt(i);
					return;
				}
			}
		}
		
		public bool IsEmpty(){
			
			if(elementSet != null){
				return (elementSet.Count <= 0);
			}
			
			return (backgroundElements.Count <= 0);
			
		}
		
	}
	
	public BackgroundEntry GetBackgroundEntryForWorldCoord(Vector3 worldCoord){
		
		if(Application.isPlaying){
			if(!worldCoordMappings.ContainsKey(worldCoord)){
				return null;
			}
			
			return worldCoordMappings[worldCoord];
		}
		else{
		
			for(int i = 0; i < backgrounds.Count; i++){
				if(backgrounds[i].ContainsWorldCoord(worldCoord)){
					return backgrounds[i];
				}
			}
	
			return null;
			
		}
		
	}
	
	public int backgroundDepth;
	public float background_x_tiling = 1.0f;
	public float background_y_tiling = 1.0f;
	
	public bool hasInitializedBase = false;
	public Vector3 baseMeshCoordinate;
	
	public bool HasBackgroundEntryFor(string name){
		
		return(GetBackgroundEntryForName(name) != null);
		
	}
	
	public BackgroundEntry GetBackgroundEntryForName(string name){
		
		for(int i = 0; i < backgrounds.Count; i++){
			
			if(backgrounds[i].backgroundObject.name == name){
				return backgrounds[i];
			}
			
		}
		
		return null;
		
	}
	
	#endregion
	
	public void AddBackground(Material material){
		
		BackgroundEntry entry = GetBackgroundEntryForName(material.name);
		
		if(entry != null){
			Debug.LogWarning("Background already exists!");
			return;
		}
						
		GameObject meshObject = null;
		
		entry = new BackgroundEntry();
		entry.backgroundMaterial = material;
				
		meshObject = new GameObject(material.name);
		meshObject.transform.parent = transform;
		meshObject.transform.localPosition = Vector3.zero;
		meshObject.transform.localRotation = Quaternion.identity;
		
		MeshFilter mf = meshObject.AddComponent<MeshFilter>();
		
		Mesh m = new Mesh();
		
		mf.mesh = m;
		
		Renderer r = meshObject.AddComponent<MeshRenderer>();
		
		r.material = material;
				
		RecalculateBackground();
		
		entry.backgroundObject = meshObject;
		entry.backgroundRenderer = r;
		entry.backgroundMeshFilter = mf;
		
		backgrounds.Add(entry);
		
	}
	
	public Mesh GetBackgroundMesh(string name){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry != null){
			MeshFilter mf = entry.backgroundMeshFilter;
			Mesh m = mf.sharedMesh;
			return m;	
		}
		else{
			Debug.LogWarning("No background entry for: " + name);
			return null;
		}
	}
	
	public void SetBackgroundMesh(Mesh m, string name){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry != null){
			MeshFilter mf = entry.backgroundMeshFilter;
			mf.sharedMesh = m;
			entry.backgroundObject.name = m.name;
		}
		else{
			Debug.LogWarning("No background entry for: " + name);
		}
		
	}
	
	public Material GetBackgroundMaterial(string name){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry == null){
			return null;
		}
		
		return entry.backgroundMaterial;
	}
	
	public void SetBackgroundMaterial(Material m){
		
		BackgroundEntry entry = GetBackgroundEntryForName(m.name);
		
		if(entry != null){
			entry.backgroundRenderer.sharedMaterial = m;
			RecalculateBackground();
		}
		else{
			Debug.LogWarning("No background to which to set a material.");
		}
	}
	
	public void RemoveBackground(bool destroyImmediate){
		
		for(int i =0; i < backgrounds.Count; i++){
			
			if(destroyImmediate){
				GameObject.DestroyImmediate(backgrounds[i].backgroundObject);
			}
			else{
				GameObject.Destroy(backgrounds[i].backgroundObject);
			}
		}
		
		backgrounds.Clear();
	}
	
	public void AddToBackgroundByPosition(Vector3[] mapPositions, string name){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry == null){
			
			Debug.LogWarning("No background exists for: " + name);
			
			return;
		}
				
		//Given this map coordinate, figure out the verts and triangles, and add them to the thing if needed
		MeshFilter mf = entry.backgroundMeshFilter;
		Mesh m = mf.sharedMesh;
		
		//Step one: Decide on the coordinates
		
		List<Vector3> vertices = new List<Vector3>(m.vertices);
		List<Vector2> UV = new List<Vector2>(m.uv);
		List<int> triangles = new List<int>(m.triangles);
				
		for(int mc = 0; mc < mapPositions.Length; mc++){
		
			Vector3 wCoords = mapPositions[mc];
			
			//Remove this from whatever background it is in
			BackgroundEntry previous = GetBackgroundEntryForWorldCoord(wCoords);
			
			if(previous == entry){
				continue;
			}
			else{
				RemoveFromBackgroundByPosition(wCoords,previous,false);
			}
			
			if(!hasInitializedBase){
				baseMeshCoordinate = wCoords;	
				hasInitializedBase = true;
			}
			
			float left,right,top,bottom;
			
			left = wCoords.x - (0.5f * tileScale.x);
			right = wCoords.x + (0.5f * tileScale.x);
			
			Vector3 vULeft;
			Vector3 vBLeft;
			Vector3 vURight;
			Vector3 vBRight;
			
			if(growthAxis == BlockMap.GrowthAxis.Forward){
				top = wCoords.z - 0.5f * tileScale.z;
				bottom = wCoords.z + 0.5f * tileScale.z;
				
				vULeft = new Vector3(left,-0.5f*tileScale.y,top);
				vBLeft = new Vector3(left,-0.5f*tileScale.y,bottom);
				vURight = new Vector3(right,-0.5f*tileScale.y,top);
				vBRight = new Vector3(right,-0.5f*tileScale.y,bottom);
			}
			else{
				top = wCoords.y + 0.5f * tileScale.y;
				bottom = wCoords.y - 0.5f * tileScale.y;
				
				vULeft = new Vector3(left,top,-0.5f*tileScale.z);
				vBLeft = new Vector3(left,bottom,-0.5f*tileScale.z);
				vURight = new Vector3(right,top,-0.5f*tileScale.z);
				vBRight = new Vector3(right,bottom,-0.5f*tileScale.z);
			}
			
			//Step two: Check if the coordinates already exist in the mesh. if so, get their index
			
			int ulIndex = -1,urIndex = -1,blIndex = -1,brIndex = -1;
				
			
			if(entry.vDictionary.ContainsKey(vULeft)){
				ulIndex = entry.vDictionary[vULeft];
			}
			
			if(entry.vDictionary.ContainsKey(vBLeft)){
				blIndex = entry.vDictionary[vBLeft];
			}
			
			if(entry.vDictionary.ContainsKey(vURight)){
				urIndex = entry.vDictionary[vURight];
			}
			
			if(entry.vDictionary.ContainsKey(vBRight)){
				brIndex = entry.vDictionary[vBRight];
			}
			
			//Step two.point.five: if not, add a vertex to the mesh and record its index (we need 4)
									
			
			//UV wise, we need to base this upon the surrounding vertices
			//If they exist already, get their UV from the list, compare the vertex to this one
			//And offset against it accordingly
					
						
			Vector2 uvULeft = Vector2.zero, uvURight = Vector2.zero, uvBLeft = Vector2.zero,uvBRight = Vector2.zero;
			
			float x_dif,y_dif;
			
			x_dif = wCoords.x - baseMeshCoordinate.x;
			
			if(growthAxis == BlockMap.GrowthAxis.Up){
				y_dif = wCoords.y - baseMeshCoordinate.y;
				uvBLeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling);
				uvULeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling + (background_y_tiling));
				uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling);
				uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling + (background_y_tiling));
			}
			else{
				y_dif = wCoords.z - baseMeshCoordinate.z;
				uvBLeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling));
				uvULeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling - (background_y_tiling)));
				uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling));
				uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling - (background_y_tiling)));
			
			}	
			
			//Debug.Log("For base coord: " + baseMeshCoordinate.ToString() + " and wcoord: " + wCoords.ToString() + " - Map: " + mapCoordinate.ToString() + ": " + uvBLeft.ToString() + "," + uvULeft.ToString() + "," + uvURight.ToString() + "," + uvBRight.ToString());
						
			List<Vector3> nVert = new List<Vector3>();
			List<int> indices = new List<int>();
			
			if(ulIndex == -1){
				
				vertices.Add(vULeft);
				nVert.Add(vULeft);
				
				UV.Add(uvULeft);
				ulIndex = vertices.Count-1;
				indices.Add(ulIndex);
				
			}
			
			if(urIndex == -1){
				
				vertices.Add(vURight);		
				nVert.Add(vURight);
				
				UV.Add(uvURight);
				urIndex = vertices.Count-1;
				indices.Add(urIndex);
				
			}
			
			if(blIndex == -1){
				vertices.Add(vBLeft);
				nVert.Add(vBLeft);
				
				UV.Add(uvBLeft);
				blIndex = vertices.Count-1;
				indices.Add(blIndex);
			}
			
			if(brIndex == -1){
				vertices.Add(vBRight);
				nVert.Add(vBRight);
				
				UV.Add(uvBRight);
				brIndex = vertices.Count-1;
				indices.Add(brIndex);
			}
			//Step three: create the triangles from these
						
			triangles.Add(blIndex);
			triangles.Add(brIndex);
			triangles.Add(ulIndex);
			
			triangles.Add(ulIndex);
			triangles.Add(brIndex);
			triangles.Add(urIndex);
				
			entry.AddCoordinate(wCoords);
			
			AddToDictionaries(nVert,indices,entry);
			
		}
			
		//Step four: add all of this to the mesh
		
		m.Clear();
		
		m.vertices = vertices.ToArray();
		m.uv = UV.ToArray();
		m.triangles = triangles.ToArray();
		
		m.RecalculateNormals();
		
		
		RefreshDictionaries(entry,m.vertices);
		
	}
	
	void AddToDictionaries(List<Vector3> vertices, List<int> indices, BackgroundEntry entry){
		
		for(int i = 0; i < vertices.Count; i++){
			entry.vDictionary.Add(vertices[i],indices[i]);
			worldCoordMappings.Add(vertices[i],entry);
		}
				
	}
	
	void RefreshDictionaries(BackgroundEntry entry, Vector3[] vertices){
		
		entry.vDictionary.Clear();
		
		for(int i = 0; i < vertices.Length; i++){
						
			entry.vDictionary.Add(vertices[i],i);
		}
	
				
	}
		
	public void AddEntryToBackground(Vector3[] mapCoordinates, string name){
		AddEntryToBackground(mapCoordinates,name,true);
	}
	
	public void AddEntryToBackground(Vector3[] mapCoordinates, string name, bool refresh){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry == null){
			
			Debug.LogWarning("No background exists for: " + name);
			
			return;
		}
	
		for(int mc = 0; mc < mapCoordinates.Length; mc++){
		
			Vector3 mapCoordinate = mapCoordinates[mc];
			
			Vector3 wCoords = BlockUtilities.GetMathematicalPosition(this,(int)mapCoordinate.x,(int)mapCoordinate.y,(int)(int)mapCoordinate.z);
			
			//Remove this from whatever background it is in
			BackgroundEntry previous = GetBackgroundEntryForWorldCoord(wCoords);
			
			if(previous == entry){
				continue;
			}
			else{
				RemoveFromBackgroundByPosition(wCoords,previous,refresh);
			}
			
			entry.AddCoordinate(wCoords);
		}
	}
	
	public void AddToBackground(Vector3[] mapCoordinates, string name){
			
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry == null){
			
			Debug.LogWarning("No background exists for: " + name);
			
			return;
		}
				
		//Given this map coordinate, figure out the verts and triangles, and add them to the thing if needed
		MeshFilter mf = entry.backgroundMeshFilter;
		Mesh m = mf.sharedMesh;
		
		//Step one: Decide on the coordinates
		
		List<Vector3> vertices = new List<Vector3>(m.vertices);
		List<Vector2> UV = new List<Vector2>(m.uv);
		List<int> triangles = new List<int>(m.triangles);

		for(int mc = 0; mc < mapCoordinates.Length; mc++){
		
			if(mapCoordinates[mc].z != backgroundDepth){
				continue;
			}
			
			Vector3 mapCoordinate = mapCoordinates[mc];
		
			Vector3 wCoords = BlockUtilities.GetMathematicalPosition(this,(int)mapCoordinate.x,(int)mapCoordinate.y,(int)(int)mapCoordinate.z);
			
			//Remove this from whatever background it is in
			BackgroundEntry previous = GetBackgroundEntryForWorldCoord(wCoords);
						
			if(previous.backgroundMaterial == entry.backgroundMaterial){
				continue;
			}
			else{
				RemoveFromBackgroundByPosition(wCoords,previous,false);
			}
			
			if(!hasInitializedBase){
				baseMeshCoordinate = wCoords;	
				hasInitializedBase = true;
			}
			
			float left,right,top,bottom;
			
			left = wCoords.x - (0.5f * tileScale.x);
			right = wCoords.x + (0.5f * tileScale.x);
			
			Vector3 vULeft;
			Vector3 vBLeft;
			Vector3 vURight;
			Vector3 vBRight;
			
			if(growthAxis == BlockMap.GrowthAxis.Forward){
				top = wCoords.z - 0.5f * tileScale.z;
				bottom = wCoords.z + 0.5f * tileScale.z;
				
				vULeft = new Vector3(left,-0.5f*tileScale.y,top);
				vBLeft = new Vector3(left,-0.5f*tileScale.y,bottom);
				vURight = new Vector3(right,-0.5f*tileScale.y,top);
				vBRight = new Vector3(right,-0.5f*tileScale.y,bottom);
			}
			else{
				top = wCoords.y + 0.5f * tileScale.y;
				bottom = wCoords.y - 0.5f * tileScale.y;
				
				vULeft = new Vector3(left,top,-0.5f*tileScale.z);
				vBLeft = new Vector3(left,bottom,-0.5f*tileScale.z);
				vURight = new Vector3(right,top,-0.5f*tileScale.z);
				vBRight = new Vector3(right,bottom,-0.5f*tileScale.z);
			}
			
			//Step two: Check if the coordinates already exist in the mesh. if so, get their index
			
			int ulIndex = -1,urIndex = -1,blIndex = -1,brIndex = -1;
										
			/*for(int i = 0; i < vertices.Count; i++){
				
				if(vertices[i] == vULeft){
					ulIndex = i;
				}
				else if(vertices[i] == vBLeft){
					blIndex = i;
					
				}
				else if(vertices[i] == vURight){
					urIndex = i;
					
				}
				else if(vertices[i] == vBRight){
					brIndex = i;								
				}
				
			}*/
			
			if(entry.vDictionary.ContainsKey(vULeft)){
				ulIndex = entry.vDictionary[vULeft];
			}
			
			if(entry.vDictionary.ContainsKey(vBLeft)){
				blIndex = entry.vDictionary[vBLeft];
			}
			
			if(entry.vDictionary.ContainsKey(vURight)){
				urIndex = entry.vDictionary[vURight];
			}
			
			if(entry.vDictionary.ContainsKey(vBRight)){
				brIndex = entry.vDictionary[vBRight];
			}
			
			//Step two.point.five: if not, add a vertex to the mesh and record its index (we need 4)
									
			/*float x_rep = 1.0f / background_x_tiling;
			float y_rep = 1.0f / background_y_tiling;
			float x_r,y_r;
			
			float x = (float)wCoords.x;
			float y = (float)wCoords.y;
			float z = (float)wCoords.z;
			
			if(growthAxis == BlockMap.GrowthAxis.Up){
				x_r = x % x_rep;
				y_r = y % y_rep;
			}
			else{
				x_r = x % x_rep;
				y_r = z % y_rep;
			}
			
			float x_r_v = x_r * background_x_tiling;
			float y_r_v = y_r * background_y_tiling;
			
			x_r_v = background_x_tiling * x;
			
			
			y_r_v = background_y_tiling * y;*/
			
			//UV wise, we need to base this upon the surrounding vertices
			//If they exist already, get their UV from the list, compare the vertex to this one
			//And offset against it accordingly
					
						
			Vector2 uvULeft = Vector2.zero, uvURight = Vector2.zero, uvBLeft = Vector2.zero,uvBRight = Vector2.zero;
			
			float x_dif,y_dif;
			
			x_dif = wCoords.x - baseMeshCoordinate.x;
			
			if(growthAxis == BlockMap.GrowthAxis.Up){
				y_dif = wCoords.y - baseMeshCoordinate.y;
				uvBLeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling);
				uvULeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling + (background_y_tiling));
				uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling);
				uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling + (background_y_tiling));
			}
			else{
				y_dif = wCoords.z - baseMeshCoordinate.z;
				uvBLeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling));
				uvULeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling - (background_y_tiling)));
				uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling));
				uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling - (background_y_tiling)));
			
			}	
			
			//Debug.Log("For base coord: " + baseMeshCoordinate.ToString() + " and wcoord: " + wCoords.ToString() + " - Map: " + mapCoordinate.ToString() + ": " + uvBLeft.ToString() + "," + uvULeft.ToString() + "," + uvURight.ToString() + "," + uvBRight.ToString());
			
			/*if(ulIndex != -1){
				uvULeft = UV[ulIndex];
			}
			
			if(urIndex != -1){
				uvURight = UV[urIndex];
			}
			
			if(brIndex != -1){
				uvBRight = UV[brIndex];
			}
			
			if(blIndex != -1){
				uvBLeft = UV[blIndex];
			}*/
			
			if(ulIndex == -1){
				
				vertices.Add(vULeft);
			
				/*if(ulIndex != -1){
				}
				else
				if(urIndex != -1){
					uvULeft = new Vector2(uvURight.x - background_x_tiling,uvURight.y);
				}
				else
				if(brIndex != -1){
					uvULeft = new Vector2(uvBRight.x - background_x_tiling,uvBRight.y + background_y_tiling);
				}
				else
				if(blIndex != -1){
					uvULeft = new Vector2(uvBLeft.x,uvBLeft.y + background_y_tiling);
				}
				else{
					uvULeft = new Vector2(0.0f,background_y_tiling);
				}*/
				
				UV.Add(uvULeft);
				ulIndex = vertices.Count-1;
	
			}
			
			if(urIndex == -1){
				
				vertices.Add(vURight);		
				
				/*if(ulIndex != -1){
					uvURight = new Vector2(uvULeft.x + background_x_tiling,uvULeft.y);
				}
				else
				if(urIndex != -1){
				}
				else
				if(brIndex != -1){
					uvURight = new Vector2(uvBRight.x,uvBRight.y + background_y_tiling);
				}
				else
				if(blIndex != -1){
					uvURight = new Vector2(uvBLeft.x + background_x_tiling,uvBLeft.y + background_y_tiling);
				}
				else{
					uvURight = new Vector2(background_x_tiling,background_y_tiling);
				}*/
				
				UV.Add(uvURight);
				urIndex = vertices.Count-1;
				
			}
			
			if(blIndex == -1){
				vertices.Add(vBLeft);
			
				/*if(ulIndex != -1){
					uvBLeft = new Vector2(uvULeft.x,uvULeft.y - background_y_tiling);
				}
				else
				if(urIndex != -1){
					uvBLeft = new Vector2(uvURight.x - background_x_tiling,uvURight.y - background_y_tiling);
				}
				else
				if(brIndex != -1){
					uvBLeft = new Vector2(uvBRight.x - background_x_tiling,uvBRight.y);
				}
				else
				if(blIndex != -1){
				}
				else{
					uvBLeft = new Vector2(0.0f,0.0f);
				}*/
				
				UV.Add(uvBLeft);
				blIndex = vertices.Count-1;
			}
			
			if(brIndex == -1){
				vertices.Add(vBRight);
				
				/*if(ulIndex != -1){
					uvBRight = new Vector2(uvULeft.x + background_x_tiling,uvULeft.y - background_y_tiling);
				}
				else
				if(urIndex != -1){
					uvBRight = new Vector2(uvURight.x,uvURight.y - background_y_tiling);
				}
				else
				if(brIndex != -1){
				}
				else
				if(blIndex != -1){
					uvBRight = new Vector2(uvBLeft.x + background_x_tiling,uvBLeft.y);
				}
				else{
					uvBRight = new Vector2(background_x_tiling,0.0f);
				}*/
				
				UV.Add(uvBRight);
				brIndex = vertices.Count-1;
			}
			//Step three: create the triangles from these
						
			triangles.Add(blIndex);
			triangles.Add(brIndex);
			triangles.Add(ulIndex);
			
			triangles.Add(ulIndex);
			triangles.Add(brIndex);
			triangles.Add(urIndex);
				
			entry.AddCoordinate(wCoords);
			
		}
			
		//Step four: add all of this to the mesh
		
		m.Clear();
		
		m.vertices = vertices.ToArray();
		m.uv = UV.ToArray();
		m.triangles = triangles.ToArray();
		
		m.RecalculateNormals();
		
		RefreshDictionaries(entry,m.vertices);
		
	}
	
	public void AddToBackground(Vector3 mapCoordinate, string name){
		
		if(mapCoordinate.z != backgroundDepth){
			
			return;
		}
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry == null){
			
			Debug.LogWarning("No background exists for: " + name);
			
			return;
			
		}
				
		//Given this map coordinate, figure out the verts and triangles, and add them to the thing if needed
		MeshFilter mf = entry.backgroundMeshFilter;
		
		Mesh m = mf.sharedMesh;
		
		//Step one: Decide on the coordinates
		
		List<Vector3> vertices = new List<Vector3>(m.vertices);
		List<Vector2> UV = new List<Vector2>(m.uv);
		List<int> triangles = new List<int>(m.triangles);

		Vector3 wCoords = BlockUtilities.GetMathematicalPosition(this,(int)mapCoordinate.x,(int)mapCoordinate.y,(int)mapCoordinate.z);
					
		//Remove this from whatever background it is in
		BackgroundEntry previous = GetBackgroundEntryForWorldCoord(wCoords);
		
		if(previous == entry){
			
			return;
		}
		else{
			RemoveFromBackgroundByPosition(wCoords,previous,false);
		}
		
		if(!hasInitializedBase){
			baseMeshCoordinate = wCoords;	
			hasInitializedBase = true;
		}
		
		float left,right,top,bottom;
		
		left = wCoords.x - (0.5f * tileScale.x);
		right = wCoords.x + (0.5f * tileScale.x);
		
		Vector3 vULeft;
		Vector3 vBLeft;
		Vector3 vURight;
		Vector3 vBRight;
		
		if(growthAxis == BlockMap.GrowthAxis.Forward){
			top = wCoords.z - 0.5f * tileScale.z;
			bottom = wCoords.z + 0.5f * tileScale.z;
			
			vULeft = new Vector3(left,-0.5f*tileScale.y,top);
			vBLeft = new Vector3(left,-0.5f*tileScale.y,bottom);
			vURight = new Vector3(right,-0.5f*tileScale.y,top);
			vBRight = new Vector3(right,-0.5f*tileScale.y,bottom);
		}
		else{
			top = wCoords.y + 0.5f * tileScale.y;
			bottom = wCoords.y - 0.5f * tileScale.y;
			
			vULeft = new Vector3(left,top,-0.5f*tileScale.z);
			vBLeft = new Vector3(left,bottom,-0.5f*tileScale.z);
			vURight = new Vector3(right,top,-0.5f*tileScale.z);
			vBRight = new Vector3(right,bottom,-0.5f*tileScale.z);
		}
		
		//Step two: Check if the coordinates already exist in the mesh. if so, get their index
		
		int ulIndex = -1,urIndex = -1,blIndex = -1,brIndex = -1;
						
		/*for(int i = 0; i < vertices.Count; i++){
			
			if(vertices[i] == vULeft){
				ulIndex = i;
			}
			else if(vertices[i] == vBLeft){
				blIndex = i;
				
			}
			else if(vertices[i] == vURight){
				urIndex = i;
				
			}
			else if(vertices[i] == vBRight){
				brIndex = i;								
			}
			
		}*/
		
		if(entry.vDictionary.ContainsKey(vULeft)){
			ulIndex = entry.vDictionary[vULeft];
		}
		
		if(entry.vDictionary.ContainsKey(vBLeft)){
			blIndex = entry.vDictionary[vBLeft];
		}
		
		if(entry.vDictionary.ContainsKey(vURight)){
			urIndex = entry.vDictionary[vURight];
		}
		
		if(entry.vDictionary.ContainsKey(vBRight)){
			brIndex = entry.vDictionary[vBRight];
		}
		
		//Step two.point.five: if not, add a vertex to the mesh and record its index (we need 4)
								
		/*float x_rep = 1.0f / background_x_tiling;
		float y_rep = 1.0f / background_y_tiling;
		float x_r,y_r;
		
		float x = (float)wCoords.x;
		float y = (float)wCoords.y;
		float z = (float)wCoords.z;
		
		if(growthAxis == BlockMap.GrowthAxis.Up){
			x_r = x % x_rep;
			y_r = y % y_rep;
		}
		else{
			x_r = x % x_rep;
			y_r = z % y_rep;
		}
		
		float x_r_v = x_r * background_x_tiling;
		float y_r_v = y_r * background_y_tiling;
		
		x_r_v = background_x_tiling * x;
		
		
		y_r_v = background_y_tiling * y;*/
		
		//UV wise, we need to base this upon the surrounding vertices
		//If they exist already, get their UV from the list, compare the vertex to this one
		//And offset against it accordingly
				
		Vector2 uvULeft = Vector2.zero, uvURight = Vector2.zero, uvBLeft = Vector2.zero,uvBRight = Vector2.zero;
		
		float x_dif,y_dif;
		
		x_dif = wCoords.x - baseMeshCoordinate.x;
		
		x_dif = x_dif / tileScale.x;
		
		if(growthAxis == BlockMap.GrowthAxis.Up){
			y_dif = wCoords.y - baseMeshCoordinate.y;
			
			y_dif = y_dif / tileScale.y;
			
			uvBLeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling);
			uvULeft = new Vector2(x_dif * background_x_tiling, y_dif * background_y_tiling + (background_y_tiling));
			uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling);
			uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), y_dif * background_y_tiling + (background_y_tiling));
		}
		else{
			y_dif = wCoords.z - baseMeshCoordinate.z;
			
			y_dif = y_dif / tileScale.z;
			
			uvBLeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling));
			uvULeft = new Vector2(x_dif * background_x_tiling, -(y_dif * background_y_tiling - (background_y_tiling)));
			uvBRight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling));
			uvURight = new Vector2(x_dif * background_x_tiling + (background_x_tiling), -(y_dif * background_y_tiling - (background_y_tiling)));
		}			
				
		//Debug.Log("For base coord: " + baseMeshCoordinate.ToString() + " and wcoord: " + wCoords.ToString() + " - Map: " + mapCoordinate.ToString() + ": " + uvBLeft.ToString() + "," + uvULeft.ToString() + "," + uvURight.ToString() + "," + uvBRight.ToString() + "Y_dif: " + y_dif + " X_dif: " + x_dif);
			
		/*if(ulIndex != -1){
			uvULeft = UV[ulIndex];
		}
		
		if(urIndex != -1){
			uvURight = UV[urIndex];
		}
		
		if(brIndex != -1){
			uvBRight = UV[brIndex];
		}
		
		if(blIndex != -1){
			uvBLeft = UV[blIndex];
		}*/
		
		if(ulIndex == -1){
			
			vertices.Add(vULeft);
		
			/*if(ulIndex != -1){
			}
			else
			if(urIndex != -1){
				uvULeft = new Vector2(uvURight.x - background_x_tiling,uvURight.y);
			}
			else
			if(brIndex != -1){
				uvULeft = new Vector2(uvBRight.x - background_x_tiling,uvBRight.y + background_y_tiling);
			}
			else
			if(blIndex != -1){
				uvULeft = new Vector2(uvBLeft.x,uvBLeft.y + background_y_tiling);
			}
			else{
				uvULeft = new Vector2(0.0f,background_y_tiling);
			}*/
			
			UV.Add(uvULeft);
			ulIndex = vertices.Count-1;

		}
		
		if(urIndex == -1){
			
			vertices.Add(vURight);		
			
			/*if(ulIndex != -1){
				uvURight = new Vector2(uvULeft.x + background_x_tiling,uvULeft.y);
			}
			else
			if(urIndex != -1){
			}
			else
			if(brIndex != -1){
				uvURight = new Vector2(uvBRight.x,uvBRight.y + background_y_tiling);
			}
			else
			if(blIndex != -1){
				uvURight = new Vector2(uvBLeft.x + background_x_tiling,uvBLeft.y + background_y_tiling);
			}
			else{
				uvURight = new Vector2(background_x_tiling,background_y_tiling);
			}*/
			
			UV.Add(uvURight);
			urIndex = vertices.Count-1;
			
		}
		
		if(blIndex == -1){
			vertices.Add(vBLeft);
		
			/*if(ulIndex != -1){
				uvBLeft = new Vector2(uvULeft.x,uvULeft.y - background_y_tiling);
			}
			else
			if(urIndex != -1){
				uvBLeft = new Vector2(uvURight.x - background_x_tiling,uvURight.y - background_y_tiling);
			}
			else
			if(brIndex != -1){
				uvBLeft = new Vector2(uvBRight.x - background_x_tiling,uvBRight.y);
			}
			else
			if(blIndex != -1){
			}
			else{
				uvBLeft = new Vector2(0.0f,0.0f);
			}*/
			
			UV.Add(uvBLeft);
			blIndex = vertices.Count-1;
		}
		
		if(brIndex == -1){
			vertices.Add(vBRight);
			
			/*if(ulIndex != -1){
				uvBRight = new Vector2(uvULeft.x + background_x_tiling,uvULeft.y - background_y_tiling);
			}
			else
			if(urIndex != -1){
				uvBRight = new Vector2(uvURight.x,uvURight.y - background_y_tiling);
			}
			else
			if(brIndex != -1){
			}
			else
			if(blIndex != -1){
				uvBRight = new Vector2(uvBLeft.x + background_x_tiling,uvBLeft.y);
			}
			else{
				uvBRight = new Vector2(background_x_tiling,0.0f);
			}*/
			
			UV.Add(uvBRight);
			brIndex = vertices.Count-1;
		}
		//Step three: create the triangles from these
		
		triangles.Add(blIndex);
		triangles.Add(brIndex);
		triangles.Add(ulIndex);
		
		triangles.Add(ulIndex);
		triangles.Add(brIndex);
		triangles.Add(urIndex);
		
		entry.AddCoordinate(wCoords);
		
		//Step four: add all of this to the mesh
		
		m.Clear();
		
		m.vertices = vertices.ToArray();
		m.uv = UV.ToArray();
		m.triangles = triangles.ToArray();
			
		m.RecalculateNormals();
		
		RefreshDictionaries(entry,m.vertices);
	}
	
	public List<Vector3> GetSurroundingCoordinates(Vector3 position){
		
		List<Vector3> coords = new List<Vector3>();
		
		int px = (int)position.x;
		int py = (int)position.y;
		int pz = (int)position.z;
		
		for(int x = px -1; x <= px + 1; x++){
			for(int y = py -1; y <= py + 1; y++){
				if(x == px && y == py){
					continue;
				}
		
				if(pz != backgroundDepth){
					continue;
				}
				
				if(editorMap){
					if(x < chunkWidth || x >= (currentWidth * chunkWidth) - chunkWidth){
						continue;
					}
					
					if(y < chunkHeight || y >= (currentHeight * chunkHeight) - chunkHeight){
						continue;
					}
				}
				
				if(GetChunkForBlockCoordinate(x,y,pz) == null){
					continue;
				}
				
				if(!GetChunkForBlockCoordinate(x,y,pz).Editor_IsInitialized()){
					continue;
				}
				
				
				coords.Add(new Vector3(x,y,position.z));
				
			}
		}
		
		return coords;
	}
	
	public void RecalculateBackground(){
		
		worldCoordMappings.Clear();
		
		for(int bg = 0; bg < backgrounds.Count; bg++){
			
			BackgroundEntry entry = backgrounds[bg];
			entry.vDictionary.Clear();
			
			if(entry == null){
				return;
			}
			
			//Given this map coordinate, figure out the verts and triangles, and add them to the thing if needed
			MeshFilter mf = entry.backgroundMeshFilter;
			Mesh m = mf.sharedMesh;
			
			m.Clear();
						
			Vector3[] tileCoords = entry.GetElementsArray();
			
			entry.backgroundElements = new List<Vector3>();
			if(entry.elementSet != null){
				entry.elementSet.Clear();
			}
			
			AddToBackgroundByPosition(tileCoords,entry.backgroundObject.name);
				
			RefreshCoordMappings();
		}
	}
	
	public BackgroundEntry RemoveFromBackground(Vector3 mapCoordinate){
		return RemoveFromBackground(mapCoordinate,true);
	}
	
	public BackgroundEntry RemoveFromBackground(Vector3 mapCoordinate, bool refresh){
				
		Vector3 wCoords = BlockUtilities.GetMathematicalPosition(this,(int)mapCoordinate.x,(int)mapCoordinate.y,(int)(int)mapCoordinate.z);
		
		BackgroundEntry entry = GetBackgroundEntryForWorldCoord(wCoords);
		
		RemoveFromBackgroundByPosition(wCoords,entry, refresh);
				
		return entry;
		
	}
	
	public void RemoveFromBackgroundByPosition(Vector3 wCoords, BackgroundEntry entry){
		RemoveFromBackgroundByPosition(wCoords,entry,true);
	}
	
	public void RemoveFromBackgroundByPosition(Vector3 wCoords, BackgroundEntry entry, bool refresh){
		
		if(entry == null){
			return;
		}
		
		entry.RemoveEntryFor(wCoords);
		
		if(refresh){
			
			RecalculateBackground();
			
		}
				
		
	}
	
	public bool IsBackgroundEmpty(string name){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry != null){
			
			if(!Application.isPlaying){
				if(entry.backgroundElements.Count <= 0){
					return true;
				}
			}
			else{
				if(entry.elementSet.Count <= 0){
					return true;
				}
			}
			
		}
		
		return false;
		
	}
		
	
	public void RemoveBackground(string name, bool destroyImmediate){
		
		BackgroundEntry entry = GetBackgroundEntryForName(name);
		
		if(entry != null){
			
			if(destroyImmediate){
				GameObject.DestroyImmediate(entry.backgroundObject);	
			}
			else{
				GameObject.Destroy(entry.backgroundObject);
			}
			
			for(int i= 0; i < backgrounds.Count; i++){
				
				if(backgrounds[i] == entry){
					backgrounds.RemoveAt(i);
					return;
				}
				
			}
			
		}
		
	}
	
	public void RemoveBackground(BackgroundEntry entry, bool destroyImmediate){
		
		if(entry != null){
			
			if(destroyImmediate){
				GameObject.DestroyImmediate(entry.backgroundObject);	
			}
			else{
				GameObject.Destroy(entry.backgroundObject);
			}
			
			for(int i= 0; i < backgrounds.Count; i++){
				
				if(backgrounds[i] == entry){
					backgrounds.RemoveAt(i);
					return;
				}
				
			}
			
		}
		
	}
	
	void BumpInts(int margin, List<int> list){
		
		for(int i = 0; i < list.Count; i++){
			if(list[i] >= margin){
				list[i] -= 1;
			}
		}
		
	}
	
	void Awake(){
		
		InitializeBackground();
		//Initialize the map
		InitializeMap();
		
		if(functionalOverlay != null){
			functionalOverlay.InitializeFunctionalOverlay();
		}
		
	}
	
	public void AddFunctionalOverlay(bool initializeOnAdd){
		
		if(functionalOverlay == null){
			functionalOverlay = gameObject.AddComponent<TidyFunctionalOverlay>();
			
			if(initializeOnAdd){
				functionalOverlay.InitializeFunctionalOverlay();
			}
		}
		else{
			Debug.LogWarning("Functional overlay exists! I cannot overwrite this, sir.");
		}
		
	}
	
	/// <summary>
	///Initialize the map. During editing, this will clear construction blocks.
	/// </summary>
	public void InitializeMap(){
		
		if(!hasBeenPublished && editorMap){
			
			ShowAllLayers();
		
			
			CleanMap();
		}
				
		OnInitializeMap();
	}
	
	public void ShowAllLayers(){
		
		if(map == null){
			return;
		}
		
		//show all layers
		//Strip empty blocks and such
		for(int i = 0; i < map.Length; i++){
			
			if(map[i] != null){
				
				ChunkSet cs = map[i];
				
				if(cs != null){
				
					for(int mi = 0; mi < cs.chunkSet.Length; mi++){
						
						MapChunk m = cs.chunkSet[mi].chunk;
							
						if(m != null){
						
#if UNITY_4_0
							m.gameObject.SetActive(true);
#else
							m.gameObject.SetActiveRecursively(true);
#endif
							
							if(m.chunkPieces != null){
								
								for(int j = 0; j < m.chunkPieces.Length; j++){
												
									Block b = m.chunkPieces[j];
									
									if(b != null){
										b.Show();
									}
									
								}
								
							}
						}
						
					}
					
				}
				
			}
		}
		
		
	}
	
	public void RemoveChunkAt(int x, int y, int depth){
		ChunkSet cs = GetNodeAt(x,y);
		
		if(cs != null){
			
			cs.RemoveChunkAt(depth);
			
		}
	}
	
	public void ShowLayer(int layer){
		
		//show only one layer
		//show all layers
		//Strip empty blocks and such
		for(int i = 0; i < map.Length; i++){
			
			if(map[i] != null){
				
				ChunkSet cs = map[i];
				
				for(int mi = 0; mi < cs.chunkSet.Length; mi++){
					
					MapChunk m = cs.chunkSet[mi].chunk;
					
					if(cs.chunkSet[mi].depth == layer){
					
#if UNITY_4_0
						m.gameObject.SetActive(true);
#else
						m.gameObject.SetActiveRecursively(true);
#endif
						
						if(m.chunkPieces != null|| m.chunkPieces.Length == 0){
							
							for(int j = 0; j < m.chunkPieces.Length; j++){
											
								Block b = m.chunkPieces[j];
								
								b.Show();
								
							}
							
						}						
					}
					else{
						
						/*if(m.chunkPieces != null || m.chunkPieces.Length == 0){
							
							for(int j = 0; j < m.chunkPieces.Length; j++){
											
								Block b = m.chunkPieces[j];
								
								b.Hide();
								
							}
							
						}
						else{
							
						}*/
						
#if UNITY_4_0
						m.gameObject.SetActive(false);
#else
						m.gameObject.SetActiveRecursively(false);
#endif
						
					}
				}
				
			}
		}
	}
	
	/// <summary>
	///Cleans working blocks from the map. 
	/// </summary>
	public void CleanMap(){
		
		if(map == null){
			return;
		}
		
		//Strip empty blocks and such
		for(int i = 0; i < map.Length; i++){
			
			if(map[i] != null){
				
				ChunkSet cs = map[i];
				
				if(cs.chunkSet == null){
					continue;
				}
				
				for(int mi = 0; mi < cs.chunkSet.Length; mi++){
										
					if(cs.chunkSet[mi].chunk == null){
						continue;
					}
					
					MapChunk m = cs.chunkSet[mi].chunk;
									
					m.CleanChunk();
					
				}
				
			}
		}
		
	}
	
	public virtual void OnInitializeMap(){}
			
	/// <summary>
	///Creates a new map, and adds the first chunk. 
	/// </summary>
	/// <param name="mapName">
	/// The name of this map
	/// </param>
	/// <param name="chunkPrefab">
	/// The prefab from which to create a 'chunk'
	/// </param>
	public void Editor_InitializeMap(string mapName, MapChunk chunkPrefab){
		
		this.name = mapName;	
				
		Editor_AddChunkAt(0,0,0,chunkPrefab,false);
				
	}
	
	
	/// <summary>
	///Returns the MapChunk at the specified coordinates 
	/// </summary>
	/// <param name="x">
	/// The x coordinate of the target chunk
	/// </param>
	/// <param name="y">
	/// The y coordinate of the target chunk
	/// </param>
	/// <returns>
	/// The MapChunk located, if any
	/// </returns>
	public MapChunk GetChunkForIndex(int x, int y, int depth){
		
		int index = y * currentWidth + x;
		
		if(x < 0 || x >= currentWidth){
			return null;
		}
		
		if(y < 0 || y >= currentHeight){
			return null;
		}
		
		ChunkSet cs = map[index];
		
		if(cs == null || !cs.HasChunks()){
			return null;
		}
		
		return cs.GetChunkAt(depth,true);
		
	}
		
	/// <summary>
	///Returns the state of all blocks surrounding the specified coordinate, used largely for orienting blocks.
	/// </summary>
	/// <param name="x">
	/// The x coordinate of the target
	/// </param>
	/// <param name="y">
	/// The y coordinate of the target
	/// </param>
	/// <returns>
	/// A 2D bool array representing the state - where true = occupied and false = empty
	/// </returns>
	public bool[,] GetBlockStateFor(int x, int y, int depth, bool isEmpty){
		
		bool[,] blockState = new bool[3,3];
		
		int x_index = 0;
		
		int width = currentWidth * chunkWidth;
		int height = currentHeight * chunkHeight;
		
		for(int xs = x -1; xs <= x+1; xs++){
			
			int y_index = 0;
			
			if(xs < 0 || xs >= width){
				
				
				blockState[x_index,y_index] = true;
				
				x_index++;
				
				continue;
			}
			
			for(int ys = y - 1; ys <= y+1; ys++){
			
				if(ys < 0 || ys >= height){
					
					blockState[x_index,y_index] = true;
					
					y_index++;
					
					continue;
				}
				
				bool occupied = false;
				
				Block block = GetBlockAt(xs,ys,depth);
								
				if(block != null && !block.isNullBlock && !block.actAsEmptyBlock){
					occupied = true;
				}
				
				//Neg-set change
				//if(block != null && !block.isNullBlock && (block.actAsEmptyBlock && isEmpty)){
				//	occupied = true;
				//}
				
				if(isEmpty){
					occupied = !occupied;
				}
				
				blockState[x_index,y_index] = occupied;
				
				y_index++;
				
			}
			
			x_index++;
		}
		
		
		return blockState;
		
	}
	
	void PrintState(bool[,] state){
		
		string s= "";
		
		for(int py = 0; py < 3; py++){
			for(int px = 0; px < 3; px++){
				s += state[px,py] +",";
			}
			s += '\n';
		}
		
		Debug.Log(s);
		
	}
	
	#region Chunk Management
	
	
	/// <summary>
	///Initializes the chunk at the specified coordinates - populating it with blocks and adding chunks surrounding it 
	/// </summary>
	/// <param name="x">
	/// The x coordinate of the target chunk
	/// </param>
	/// <param name="y">
	/// The y coordinate of the target chunk
	/// </param>
	/// <param name="blocks">
	/// An array of instantiated blocks with which to populate the chunk
	/// </param>
	/// <param name="chunkPrefab">
	/// The prefab from which to spawn surrounding chunks
	/// </param>
	public void Editor_InitializeChunkAt(int x, int y, int depth, Block[] blocks, MapChunk chunkPrefab){
			
		if(HasChunkAt(x,y,depth)){
						
			MapChunk chunk = GetChunkAt(x,y,depth,false);
			
			chunk.Editor_InitializeChunk(blocks);
			
			chunk.BindAllBlocksToMap();
			
			for(int x1 = -1; x1 <= 1; x1++){
				for(int y1 = -1; y1 <= 1; y1++){
									
					int x_coord = chunk.x + x1;
					int y_coord = chunk.y + y1;
					
					if(!HasChunkAt(x_coord,y_coord,depth)){
						
						Editor_AddChunkAt(x_coord,y_coord,depth,chunkPrefab,false);
						
						//look around this chunk
						//if it is above or below an already-initialized chunk
						//initialize it and bind the transforms to that chunk
						
					}
				}	
			}
		}
		
	}
		
	public void Editor_AddChunkAt(int x, int y, int depth, MapChunk chunkPrefab, bool placeAbsolute){
						
		ChunkSet cs = GetNodeAt(x,y);
				
		if(depth > mapUpperDepth){
			mapUpperDepth = depth;
		}
		
		if(depth < mapLowerDepth){
			mapLowerDepth = depth;
		}
		
		MapChunk mc = null;
		
		if(cs == null){
			cs = new ChunkSet();
			
			cs.InitializeChunkSet(this,x,y);
			
			AddNodeAt(cs,x,y);
			
			mc = GameObject.Instantiate(chunkPrefab) as MapChunk;
			
			mc.name = "MapChunk_"+x+"_"+y;
						
			cs.AddChunkAt(mc,depth);
			
			mc.Editor_Activate(chunkWidth,chunkHeight,cs.x,cs.y,depth,this,placeAbsolute);
						
			return;
		}
		else{
			
			if(cs.x != x){
				cs.x = x;
			}
			
			if(cs.y != y){
				cs.y = y;
			}
			
		}
				
		if(cs.GetChunkAt(depth,false) != null){
									
			return;
		}
		
		mc = GameObject.Instantiate(chunkPrefab) as MapChunk;
			
		mc.name = "MapChunk_"+x+"_"+y;
			
		//at this point we want to do a different activation, as the chunkset already exists
		//and this is therefore at a different depth
		cs.AddChunkAt(mc,depth);
		
		int upper = depth+1;
		int lower = depth-1;
		
		MapChunk u = cs.GetChunkAt(upper,false);
		MapChunk l = cs.GetChunkAt(lower,false);
				
		if(u != null){
			
			mc.Editor_ActivateBound(chunkWidth,chunkHeight,cs.x,cs.y,depth,this,u);
			
		}
		else if(l != null){
			
			mc.Editor_ActivateBound(chunkWidth,chunkHeight,cs.x,cs.y,depth,this,l);
			
		}
		else{
			
			mc.Editor_Activate(chunkWidth,chunkHeight,cs.x,cs.y,depth,this,placeAbsolute);
		}
				
	}
	
	public MapChunk GetChunkForBlockCoordinate(int x, int y, int depth){
		
		int chunk_x = (int)((float)x / (float)chunkWidth);
		int chunk_y = (int)((float)y / (float)chunkHeight);
		
		return GetChunkAt(chunk_x,chunk_y,depth,false);
		
	}
	
	/// <summary>
	///Is there a chunk at this coordinate? 
	/// </summary>
	/// <param name="x">
	/// The x coordinate of the target chunk
	/// </param>
	/// <param name="y">
	/// The y coordinate of the target chunk
	/// </param>
	/// <returns>
	/// True if a chunk exists, false if not
	/// </returns>
	public bool HasChunkAt(int x, int y, int depth){
		
		if(GetChunkAt(x,y,depth,false) != null){
			return true;
		}
		
		return false;
	}
	
	/// <summary>
	///Return the chunk at the specified coordinates 
	/// </summary>
	/// <param name="x">
	/// The x coordinate of the target chunk
	/// </param>
	/// <param name="y">
	/// The y coordinate of the target chunk
	/// </param>
	/// <returns>
	/// The chunk, if any, at the target coordinates
	/// </returns>
	public MapChunk GetChunkAt(int x, int y, int depth, bool getClosest){
		
		ChunkSet cs = GetNodeAt(x,y);
		
		if(cs == null || !cs.HasChunks()){
			
			return null;
		}
				
		return cs.GetChunkAt(depth,getClosest);
		
	}
	
	public ChunkSet GetChunkSetAt(int x, int y){
		
		ChunkSet node = GetNodeAt(x,y);
		
		return node;
		
	}
	
	/// <summary>
	///Returns the block at the target map coordinates 
	/// </summary>
	/// <param name="x">
	///The x coordinate of the target block
	/// </param>
	/// <param name="y">
	///The y coordinate of the target block
	/// </param>
	/// <returns>
	///The block, if one exists
	/// </returns>
	public Block GetBlockAt(int x, int y, int depth){
		
		int c_x = x / chunkWidth;
		int c_y = y / chunkHeight;
		
		int b_x = x % chunkWidth;
		int b_y = y % chunkHeight;
		
		MapChunk chunk = GetChunkAt(c_x,c_y,depth,false);
		
		if(chunk != null){
			
			return chunk.GetBlockAtChunkCoord(b_x,b_y);
			
		}
		
		return null;
	}
	
	/// <summary>
	///Sets the block at the given coordinates
	/// </summary>
	/// <param name="x">
	///The x coordinate of the target block
	/// </param>
	/// <param name="y">
	///The y coordinate of the target block
	/// </param>
	/// <param name="block">
	///The instantiated block to be placed at the coordinate
	/// </param>
	public void SetBlockAt(int x, int y, int depth, Block block, bool destroyImmediate){
		
		int c_x = x / chunkWidth;
		int c_y = y / chunkHeight;
		
		int b_x = x % chunkWidth;
		int b_y = y % chunkHeight;
		
		MapChunk chunk = GetChunkAt(c_x,c_y,depth,false);
		
		if(chunk != null){
			
			if(functionalOverlay != null){
				Block b = chunk.GetBlockAtChunkCoord(b_x,b_y);
				
				if(b != null){
					if(b.name == block.name){
						TidyFunctionalObject data = functionalOverlay.GetDataAt(x,y,depth);
						data.SetParentBlock(block);
					}
					else{
						TidyFunctionalObject o = functionalOverlay.RemoveDataAt(x,y,depth);
						if(destroyImmediate){
							GameObject.DestroyImmediate(o);
						}
						else{
							GameObject.Destroy(o);
						}
					}
				}
			}
			
			chunk.SetBlockAt(b_x,b_y,block,destroyImmediate);
			
			if(block != null){
				block.BindToMap(x,y,depth,this);
			}
			
		}
		
	}
	
	/// <summary>
	///Refreshes the map - reorienting the blocks contained within it 
	/// </summary>
	public void RefreshMap(){
		
		if(map == null){
			
			//Debug.LogWarning("Attempting to refresh null map! Aborting.");
			
			return;
			
		}
		
		for(int i = 0; i < map.Length; i++){
			
			//Debug.Log("Refreshing chunk: " + i);
			
			if(map[i] != null){
				map[i].RefreshChunkSet();
			}
			
		}
		
	}
		
	#endregion
		
	#region Pathing
	
	public int GetMapWidth(){
		return (currentWidth) * chunkWidth;
	}
	
    public int GetMapHeight(){
		return (currentHeight) * chunkHeight;
	}
	
    public bool IsWalkable(int x, int y, int z, bool pathOverAllBlocks){
		Block b = GetBlockAt(x,y,z);
		
		MapChunk c= GetChunkForBlockCoordinate(x,y,z);
		
		if(c == null || !c.Editor_IsInitialized()){
			return false;
		}
		
		if(b != null && !b.isNullBlock && !b.actAsEmptyBlock && !pathOverAllBlocks){
			return false;
		}
		
		return true;
	}
	
	#endregion
	
	#region Various bits
	
	public MapChunk GetLeftMostMapChunk(){
		
		for(int y = 0; y < GetMapHeight(); y++){
			
			MapChunk m = GetChunkAt(0,y,0,false);
			
			if(m != null){
				return m;
			}
			
		}
		
		return null;
		
	}
	
	public MapChunk GetTopMostMapChunk(){
		
		for(int x = 0; x < GetMapWidth(); x++){
			
			MapChunk m = GetChunkAt(x,0,0,false);
			
			if(m != null){
				return m;
			}
			
		}
		
		return null;
		
	}
	
	#endregion
}


