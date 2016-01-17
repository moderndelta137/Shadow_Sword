using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using DopplerInteractive.TidyTileMapper.IconManagement;
using DopplerInteractive.TidyTileMapper.Utility;
using DopplerInteractive.TidyTileMapper.Words;
using DopplerInteractive.TidyTileMapper.Layering;
using DopplerInteractive.TidyTileMapper.Utilities.Pathing;
using System.Reflection;
using DopplerInteractive.TidyTileMapper.FunctionalOverlay;

namespace DopplerInteractive.TidyTileMapper.Editors{
	
	//Rebuild please
	
	public enum MapPaintBehaviour{
		Disabled,
		Paint,
		Cycle,
		Block_Move,
		Add_Layer_Above,
		Add_Layer_Below,
		DrawPath,
		Delete_Chunk,
		Edit_Functions,
		Paint_Background,
		Plugin_Active
	};
	
	public enum StrippingLevel{
		Working_Blocks_Only,
		Strip_Empty_Triggers,
		Strip_All
	};
	
	public class TidyBlockMapCreator : IEditorWindow
	{	
		MapPaintBehaviour currentBehaviour;
		
		bool triggerSelection = false;
		//GameObject previousSelection = null;
		//int updateCount = 0;
				
		public void TriggerSelection(){
			
			return;
			
			/*previousSelection = Selection.activeGameObject;
			
			Selection.activeGameObject = focusMap.gameObject;
			
			triggerSelection = true;
			
			updateCount = 0;*/
			
		}
		
		public void RevertSelection(){
			
			return;
			
			/*updateCount++;
			
			if(updateCount <= 1){
				return;
			}
			
			Selection.activeGameObject = previousSelection;
			triggerSelection = false;
			
			updateCount = 0;*/
		}
		
		public void SetPaintBehaviour(MapPaintBehaviour behaviour){
						
			
			leftMouseDown = false;
			
			if(currentBehaviour == behaviour){
				return;
			}
				
			if(currentBehaviour == MapPaintBehaviour.Edit_Functions){
				functionData = null;
				functionBlock = null;
				prop = null;
			}
			
			if(behaviour == MapPaintBehaviour.Edit_Functions){
				RefreshFunctionalObjects();
				RefreshExistentBlockMaps();
			}
			
			if(currentBehaviour == MapPaintBehaviour.DrawPath){
				drawingPath = false;
				pathBlocks.Clear();
			}
			
			currentBehaviour = behaviour;
			
			if(currentBehaviour == MapPaintBehaviour.Block_Move){
					
				if(selectedBlock != null){
					Selection.activeGameObject = selectedBlock.gameObject;
				}
			}
						
			SceneView.RepaintAll();
			
			if(currentBehaviour == MapPaintBehaviour.Disabled){
				
				/*if(plugins != null){
					for(int i = 0; i < plugins.Length; i++){
						plugins[i].obj.DeactivateTool();
					}
				}*/
				
			}
			
		}
		
		bool foldAdvancedOptions = false;
		bool foldMapOptions = false;
		
		OrientedBlock workingBlock = null;
		
		//Editor Logic
		EditorWindow parentWindow;

		Texture2D itemSelectedTexture = null;
		
		Texture2D lineTexture = null;
		
		string newMapName = "";
		
		string mapCreationMessage;
		
		//Icon and preview management
		PreviewDictionary previews;
		TextureDictionary icons;
		
		float tileWidth = 1.0f;
		float tileHeight = 1.0f;
		float tileDepth = 1.0f;
		
		int chunkWidth = 5;
		int chunkHeight = 5;
		
		StrippingLevel selectedStrippingLevel;
			
		BlockMap.GrowthAxis growthAxis;
		
		//GUI Logic
		float idealBlockIconWidth = 50.0f;
		
		float maximumButtonWidth = 50.0f;
		
		Vector2 blockWindowScrollPos = Vector2.zero;
		
		List<OrientedBlock> workingBlocks = null;
		
		GUIStyle italicStyle;
				
		string mapPublishStatus = "";
		
		BlockMap[] existentBlockMaps;
		
		public OrientedBlock GetWorkingBlock(){
			return workingBlock;
		}
		
		public void SetWorkingBlockNonEmpty(){
			
			if(workingBlock.isNullBlock){
				if(workingBlocks.Count > 0){
					workingBlock = workingBlocks[1];
				}
			}
		}
				
		#region IEditorWindow implementation
		public void Initialize(EditorWindow parentWindow){
			
			this.parentWindow = parentWindow;
			
			/*Rect wSize = this.parentWindow.position;
			
			wSize.width = 220.0f;
			
			this.parentWindow.position = wSize;*/
						
			this.parentWindow.name = TidyMessages.MAP_CREATOR_WINDOW_NAME;
								
			//Moving this to public
			//SceneView.onSceneGUIDelegate += DrawScene;
		
			itemSelectedTexture = GetIcon(TidyFileNames.ICON_ITEM_SELECTED);
			
			if(itemSelectedTexture == null){
				
				itemSelectedTexture = EditorGUIUtility.whiteTexture;

			}
			
			lineTexture = GetIcon(TidyFileNames.GUI_LINE);
			
			if(lineTexture == null){
				lineTexture = EditorGUIUtility.whiteTexture;
			}
			
			mapCreationMessage = TidyMessages.MAP_CREATOR_DEFAULT_MAP_MESSAGE;
			
			mapPublishStatus = TidyMessages.MAP_CREATOR_DEFAULT_PUBLISH_MESSAGE;
			
			PopulateBlockList();
			
			InitializePlugins();
			
			try{
				//Set up our italic style
				italicStyle = new GUIStyle(EditorStyles.label);
						
				italicStyle.fontStyle = FontStyle.Italic;
				
												
			}catch(Exception e){
				e.ToString();
			}
			
			RefreshExistentBlockMaps();
			
			LoadBackgroundMaterials();
			
		}
		
		void PopulateBlockList(){
			
			Block[] b = TidyEditorUtility.GetCurrentBlocks();
						
			workingBlocks = new List<OrientedBlock>();
			
			if(b != null){
				
				for(int i = 0; i < b.Length; i++){
					
					if(b[i].GetDefaultBlock() != null){
						workingBlocks.Add(b[i] as OrientedBlock);
					}
					
				}
				
			}
			
			workingBlocks.Insert(0,TidyEditorUtility.GetNullBlock() as OrientedBlock);
			
			//Working block
			if(workingBlocks.Count > 0){
				
				workingBlock = workingBlocks[0];
		
			}
		}
		
		public void RefreshWorkingBlockMenu(){
			
			//Debug.Log("Refresh working block");
			
			workingBlocks.Clear();
			
			TidyEditorUtility.RefreshCurrentBlockList();
			
			PopulateBlockList();
			
			parentWindow.Repaint();
			
			RefreshAllMaps();
			
		}
		
		public void RefreshAllMaps(){
			
			//We need to refresh all blocks in case something has changed
			BlockMap[] allMaps = GameObject.FindObjectsOfType(typeof(BlockMap)) as BlockMap[];
			
			for(int i = 0; i < allMaps.Length; i++){
				
				if(allMaps[i] == null){
					continue;
				}
								
				allMaps[i].RefreshMap();
				
				if(allMaps[i].map == null){
					
					continue;
				}
				
				for(int j = 0; j < allMaps[i].map.Length; j++){
										
					if(allMaps[i].map[j] == null){
						
						continue;
						
					}
					
					ChunkSet cs = allMaps[i].map[j];
					
					if(cs.chunkSet == null){
						
						continue;
					}
					
					for(int c = 0; c < cs.chunkSet.Length; c++){
							
						if(cs.chunkSet[c] == null){
							continue;
						}
						
						if(cs.chunkSet[c].chunk == null){
							continue;
						}
						
						MapChunk mc = cs.chunkSet[c].chunk;
						
						if(mc.chunkPieces == null){
							continue;
						}
						
						for(int cp = 0; cp < mc.chunkPieces.Length; cp++){
							
							if(mc.chunkPieces[cp] == null){
								continue;								
							}
							
							Block b = mc.chunkPieces[cp];
							
							SetBlockDirty(b);
							
							EditorUtility.SetDirty(mc.chunkPieces[cp]);
						}
						
					}
					
				}
								
				SetEntireMapDirty(allMaps[i]);
								
			}
		}
		
		float lastAction = 0.0f;
		float actionRate = 0.15f;
		
		bool canAct = true;
		
		DateTime lastFrame = DateTime.Now;
		
		public void Update(){
						
			if(selectionChangeRequired){
				
				selectionChangeRequired = false;
				
				Selection.activeGameObject = selectedBlock.gameObject;
				
			}
			
			if(triggerSelection){
				RevertSelection();
			}
			
			//We need to update this until the sync is finished
			if(UpdatePreviews()){
					
				//Debug.Log("Updating previews");
				
				parentWindow.Repaint();
			}
			
			DateTime currentTime = DateTime.Now;
			
			UpdateActionTimer((float)(currentTime.Subtract(lastFrame).TotalMilliseconds / 1000.0f));
			
			lastFrame = currentTime;
			
		}
		
		void UpdateActionTimer(float deltaTime){
						
			if(canAct){
				return;
			}
				
			lastAction += deltaTime;
			
			if(lastAction >= actionRate){
				lastAction = 0.0f;
				canAct = true;
			}
			
		}
		
		void HasActed(){
			
			canAct = false;
			
		}
				
		public void DrawFunctionalOverlayArea(){
			
			return;
			
			/*DrawHorizontalLine();
			
			GUILayout.Label("Functional Overlay:");
			
			GUILayout.Space(10.0f);
			
			if(focusMap == null){
				GUILayout.Label("Select a map to which to add a functional overlay.");
			}
			else{
				if(focusMap.functionalOverlay == null){
					
					EditorGUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					
					if(GUILayout.Button("Add overlay")){
						focusMap.AddFunctionalOverlay(false);
						SetEntireMapDirty(focusMap);
					}
					
					GUILayout.Space(20.0f);
					
					EditorGUILayout.EndHorizontal();
					
				}
				else{
					
					GUILayout.Label(focusMap.name + " contains functional overlay.");
					
				}
			}
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();*/
		}
		
		public bool MapNameExists(string name){
			
			if(existentBlockMaps == null){
				return false;
			}
			
			for(int i = 0; i < existentBlockMaps.Length; i++){
				if(existentBlockMaps[i].name == name){
					return true;
				}
			}
			
			return false;
			
		}
		
		Vector2 windowScrollPos = Vector2.zero;
		
		int tilesAcross;
		float extraBit;
		
		public void DrawWindow ()
		{				
			if(existentBlockMaps == null){
				
				RefreshExistentBlockMaps();
			}
			
			if(workingBlocks == null){
				
				PopulateBlockList();
			}
			
			if(italicStyle == null){
					
				//Set up our italic style
				italicStyle = new GUIStyle(EditorStyles.label);
				
				italicStyle.fontStyle = FontStyle.Italic;
				
			}
			
			
			//Color c = GUI.color;
			
			float width = (parentWindow.position.width + 20.0f);
			
			if(width <= idealBlockIconWidth){
				tilesAcross = 1;
				extraBit = width - idealBlockIconWidth;
			}
			else{
				tilesAcross = (int)((width) / idealBlockIconWidth);
				extraBit = ((width) % (float) idealBlockIconWidth) / (float)tilesAcross;
				
				if(tilesAcross == 0){
					tilesAcross = 1;
					extraBit = width - idealBlockIconWidth;
				}
			}
			
			windowScrollPos = EditorGUILayout.BeginScrollView(windowScrollPos);
											
			EditorGUILayout.BeginVertical();
			
			GUILayout.Space(10.0f);
						
			GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_CATEGORY,EditorStyles.boldLabel);
			
			EditorGUILayout.BeginHorizontal();
			
			GUILayout.FlexibleSpace();
			
			if(GUILayout.Button(new GUIContent(TidyMessages.MAP_CREATOR_BLOCK_EDITOR,TidyTooltips.MAP_CREATOR_BLOCK_EDITOR))){
									
				EditorApplication.ExecuteMenuItem("Window/Tidy Block Editor");
								
			}
			
			GUILayout.Space(20.0f);
			
			EditorGUILayout.EndHorizontal();
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_CREATE_MAP_CATEGORY,EditorStyles.boldLabel);
			
			GUILayout.Space(10.0f);
						
			if(currentBehaviour != MapPaintBehaviour.Disabled){
				GUI.enabled = false;
			}
			
			if(TidyEditorUtility.GetMapChunkPrefab() != null){
				
				EditorGUILayout.BeginHorizontal();
				
				GUILayout.Label(TidyMessages.MAP_CREATOR_NEW_MAP_NAME,GUILayout.ExpandWidth(false));
				
				newMapName = GUILayout.TextField(newMapName,GUILayout.ExpandWidth(true));
				
				EditorGUILayout.EndHorizontal();
				
				foldMapOptions = EditorGUILayout.Foldout(foldMapOptions,TidyMessages.MAP_CREATOR_ADVANCED_MAP_OPTIONS);
				
				if(foldMapOptions){
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUI.indentLevel = 1;
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_TILE_WIDTH);
					
					GUILayout.FlexibleSpace();
					
					tileWidth = EditorGUILayout.FloatField(tileWidth,GUILayout.ExpandWidth(false));
					
					if(tileWidth <= 0.0f){
						tileWidth = 1.0f;
					}
					
					EditorGUILayout.EndHorizontal();
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_TILE_HEIGHT);
					
					GUILayout.FlexibleSpace();
					
					tileHeight = EditorGUILayout.FloatField(tileHeight,GUILayout.ExpandWidth(false));
					
					if(tileHeight <= 0.0f){
						tileHeight = 1.0f;
					}
					
					EditorGUILayout.EndHorizontal();
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_TILE_DEPTH);
					
					GUILayout.FlexibleSpace();
					
					tileDepth = EditorGUILayout.FloatField(tileDepth,GUILayout.ExpandWidth(false));
					
					if(tileDepth <= 0.0f){
						tileDepth = 1.0f;
					}
										
					EditorGUILayout.EndHorizontal();
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_NEW_MAP_CHUNK_WIDTH);
										
					GUILayout.FlexibleSpace();
					
					chunkWidth = EditorGUILayout.IntField(chunkWidth,GUILayout.ExpandWidth(false));
					
					if(chunkWidth <= 0){
						chunkWidth = 1;
					}
										
					EditorGUILayout.EndHorizontal();
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_NEW_MAP_CHUNK_HEIGHT);
										
					GUILayout.FlexibleSpace();
					
					chunkHeight = EditorGUILayout.IntField(chunkHeight,GUILayout.ExpandWidth(false));
					
					if(chunkHeight <= 0){
						chunkHeight = 1;
					}
										
					EditorGUILayout.EndHorizontal();
					
					EditorGUILayout.BeginHorizontal();
					
					EditorGUILayout.PrefixLabel(TidyMessages.MAP_CREATOR_MAP_GROW_AXIS);
					
					GUILayout.FlexibleSpace();
					
					growthAxis = (BlockMap.GrowthAxis)EditorGUILayout.EnumPopup(growthAxis,GUILayout.ExpandWidth(false));
					
					EditorGUILayout.EndHorizontal();
					
					EditorGUI.indentLevel = 0;
					
				}
				
				EditorGUILayout.BeginHorizontal();
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button(new GUIContent(TidyMessages.MAP_CREATOR_ADD_MAP,TidyTooltips.MAP_CREATOR_ADD_MAP))){
					
					if(workingBlock == null){
						
						mapCreationMessage = TidyMessages.MAP_CREATOR_NO_BLOCK_CHOSEN;
						
					}
					else
					if(string.IsNullOrEmpty(newMapName)){
						
						mapCreationMessage = TidyMessages.MAP_CREATOR_NO_NAME;
						
					}
					else 
					if(MapNameExists(newMapName)){
						mapCreationMessage = TidyMessages.MAP_CREATOR_NAME_EXISTS;
					}
					else{
						
						mapCreationMessage = TidyMessages.MAP_CREATOR_MAP_CREATED;
						
						CreateMap(newMapName);
						
						RefreshExistentBlockMaps();
						
					}
					
					newMapName = "";
					
				}
				
				GUILayout.Space(20.0f);
				
				EditorGUILayout.EndHorizontal();
						
				GUILayout.Space(10.0f);
				
								
			}
			else{
				GUILayout.Label(TidyMessages.MAP_CREATOR_NO_CHUNK_ERROR);
			}
			
			
			GUILayout.Space(10.0f);
			
			GUILayout.Label(mapCreationMessage,italicStyle);
			
			if(currentBehaviour != MapPaintBehaviour.Disabled){
				GUI.enabled = true;
			}
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_STREAMING_MAPS,EditorStyles.boldLabel);
						
			if(GUILayout.Button(TidyMessages.MAP_CREATOR_ADD_STREAMING_MAP)){
				
				//Add a streaming map to the scene
				GameObject newStreamingMap = new GameObject("Streaming Map");
				
				//We are doing this by name and not by type because...
				//I am currently working in a C# project
				//To be compiled to a DLL
				//(For good but complex reasons)
				//And StreamingMap is in another project
				
				bool success = true;
				
				try{
					newStreamingMap.AddComponent<StreamingMap>();
				}catch(Exception e){
					
					Editor.DestroyImmediate(newStreamingMap);
					success = false;
					Debug.LogError("When creating streaming map:\n" + e.ToString());
				}
				
				if(success){
					
					EditorUtility.SetDirty(newStreamingMap.gameObject);
					Selection.activeGameObject = newStreamingMap;
					
				}
			}
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			DrawMapInfo();
			
			GUILayout.Space(10.0f);
			
			DrawFunctionalOverlayArea();
			
			DrawMapTools();
						
			DrawToolOptions();
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_LIST);
			
			int rows = workingBlocks.Count / tilesAcross + 1;
			
			blockWindowScrollPos = EditorGUILayout.BeginScrollView(blockWindowScrollPos,GUILayout.Width(parentWindow.position.width),GUILayout.MinHeight(idealBlockIconWidth * rows + 10.0f));
			
			Rect scrollVertRect = EditorGUILayout.BeginVertical();
		
			GUI.Box(scrollVertRect,"");
			
			for(int i = 0; i < workingBlocks.Count; i++){
				
				if(workingBlocks[i] == null){
					RefreshWorkingBlockMenu();
					continue;
				}
				
				if(i % tilesAcross == 0){
					
					if(i > 0){
						GUILayout.FlexibleSpace();
						EditorGUILayout.EndHorizontal();
					}
					
					EditorGUILayout.BeginHorizontal();
					
				}
				
				if(GUILayout.Button(new GUIContent("",workingBlocks[i].name),GUILayout.Width(idealBlockIconWidth+extraBit),GUILayout.Height(idealBlockIconWidth+extraBit))){
					workingBlock = workingBlocks[i];
				}
				
				GameObject pObject = workingBlocks[i].GetDefaultBlock();
				
				if(pObject == null){
					pObject = workingBlocks[i].gameObject;
				}
				
				Texture2D texture = GetPreviewForGameObject(pObject,false);				
				
				if(texture != null){
					GUI.DrawTexture(GUILayoutUtility.GetLastRect(),texture,ScaleMode.ScaleAndCrop,false);
				}
				
				if(workingBlocks[i] == workingBlock){
					GUI.DrawTexture(GUILayoutUtility.GetLastRect(),itemSelectedTexture,ScaleMode.ScaleToFit,true);	
				}
				
				if(i == workingBlocks.Count-1){
					GUILayout.FlexibleSpace();
					EditorGUILayout.EndHorizontal();
					
				}
				
			}
			
			EditorGUILayout.EndVertical();
						
			EditorGUILayout.EndScrollView();
			
			EditorGUILayout.BeginHorizontal();
			
			GUILayout.FlexibleSpace();
			
			if(GUILayout.Button("Refresh")){
				
				RefreshWorkingBlockMenu();
				
			}
			
			GUILayout.Space(20.0f);
			
			EditorGUILayout.EndHorizontal();
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			DrawBackgroundPanel();
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			GUILayout.Space(10.0f);
			
			DrawPlugins();
			
			/*GUILayout.Space(10.0f);
			
			DrawHorizontalLine();*/
			
			DrawMapVisibilityPanel();
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();
			
			GUILayout.Space(10.0f);
			
			foldAdvancedOptions = EditorGUILayout.Foldout(foldAdvancedOptions,TidyMessages.MAP_CREATOR_TIDY_ADVANCED_OPTIONS);
			
			if(foldAdvancedOptions){
				
				DrawAdvancedOptions();
				
			}
			
			GUILayout.Space(10.0f);
			
			DrawHorizontalLine();			
			
			GUILayout.FlexibleSpace();
			
			EditorGUILayout.EndVertical();
			
			EditorGUILayout.EndScrollView();
												
		}
		
		//ReflectedPlugin[] plugins = null;
		
		class ReflectedPlugin{
			
			public Type type;
			//public MethodInfo method;
			public ITTMPlugin obj;
			public Assembly assembly;
			
			public ReflectedPlugin(string typeName, string methodName, Assembly assembly,TidyBlockMapCreator mapCreator){
				
				this.assembly = assembly;
				
				this.type = assembly.GetType(typeName);
				
				if(this.type == null){
					
					Debug.LogWarning("Could not find type: " + typeName + " in assembly: " + assembly.FullName);
					
					return;
				}
								
				obj = Activator.CreateInstance(type) as ITTMPlugin;
				
				if(obj == null){
					Debug.LogWarning("Could not create ITTMPlugin from type: " + typeName + " in assembly: " + assembly.FullName);
					return;
				}
				
				obj.InitializePlugin(mapCreator.parentWindow,mapCreator);
			}
			
		}
		
		void InitializePlugins(){
			
			/*Assembly[] assemblies = TidyEditorUtility.GetAllPlugins();
			
			plugins = new ReflectedPlugin[assemblies.Length];
			
			for(int i = 0; i < assemblies.Length; i++){
				
				plugins[i] = new TidyBlockMapCreator.ReflectedPlugin(TidyFileNames.PLUGIN_MAIN_CLASS_NAME,TidyFileNames.PLUGIN_MAIN_METHOD,assemblies[i],this);
				
			}*/
			
		}
		
		void DrawPlugins(){
			
			/*GUILayout.Label("Plugins",EditorStyles.boldLabel);
			
			if(plugins == null || plugins.Length == 0){
				
				GUILayout.Label("No plugins available",italicStyle);
				
			}
			else{
				
				for(int i = 0; i < plugins.Length; i++){
					
					if(plugins[i] == null){
						continue;
					}
					
					if(plugins[i].obj == null){
						continue;
					}
					
					plugins[i].obj.DrawPlugin();
					
				}
				
			}*/
				
			
			
		}
		
		public void InspectorUpdate(){
			if(shouldPublish && hasUpdated){
				
				PublishMap();
				
			}	
			
			if(shouldPublish && !hasUpdated){
				hasUpdated = true;
			}
		}
		
		bool foldOptions = true;
		
		bool paintRandom = false;
		
		void DrawToolOptions(){

			switch(currentBehaviour){
								
				case MapPaintBehaviour.Cycle:{
				
					break;
				}
				case MapPaintBehaviour.Paint:{
				
					foldOptions = EditorGUILayout.Foldout(foldOptions,TidyMessages.MAP_CREATOR_PAINT_OPTIONS);
				
					if(foldOptions){
				
						EditorGUILayout.BeginHorizontal();
					
						EditorGUI.indentLevel = 2;
					
						paintRandom = EditorGUILayout.Toggle(TidyMessages.MAP_CREATOR_PAINT_RANDOM,paintRandom);
						
						EditorGUI.indentLevel = 0;
					
						EditorGUILayout.EndHorizontal();
					
					}
					break;
				}
				case MapPaintBehaviour.Disabled:{
				
					break;
				}
				
				case MapPaintBehaviour.Block_Move:{
				
					EditorGUILayout.BeginVertical();
				
					foldBlockMoveTools = EditorGUILayout.Foldout(foldBlockMoveTools,TidyMessages.MAP_CREATOR_BLOCK_MOVE_OPTIONS);
					
					if(foldBlockMoveTools){
					
						EditorGUI.indentLevel = 2;
						
						blockMoveRoundness = EditorGUILayout.Slider(TidyMessages.MAP_CREATOR_BLOCK_MOVE_ROUNDNESS,blockMoveRoundness,0.0f,1.0f);
						blockSelectionRadius = EditorGUILayout.IntField(TidyMessages.MAP_CREATOR_BLOCK_MOVE_RADIUS,blockSelectionRadius);
						
						if(blockSelectionRadius <= 0){
							blockSelectionRadius = 0;
						}
						
						block_move_selection = (Block_Move_Selection)EditorGUILayout.EnumPopup(TidyMessages.MAP_CREATOR_BLOCK_MOVE_RADIUS_TYPE,block_move_selection);
						
						EditorGUI.indentLevel = 0;
					}
						
					
				
						break;
				}
				
			case MapPaintBehaviour.DrawPath:{
				
				foldOptions = EditorGUILayout.Foldout(foldOptions,TidyMessages.MAP_CREATOR_PAINT_OPTIONS);
				
					if(foldOptions){
					
						EditorGUI.indentLevel = 2;
														
						pathWidth = EditorGUILayout.IntField(new GUIContent(TidyMessages.MAP_CREATOR_PATH_WIDTH,
					                                                    null,
					                                                    TidyTooltips.MAP_CREATOR_PATH_WIDTH
					                                                    ),pathWidth);
					
						if(pathWidth <= 0){
						
							pathWidth = 1;
						
						}
					
						pathRandomisation = EditorGUILayout.Toggle(new GUIContent(TidyMessages.MAP_CREATOR_PATH_RANDOMISATION,
					                                                    null,
					                                                    TidyTooltips.MAP_CREATOR_PATH_RANDOMISATION
					                                                    ),pathRandomisation);
				
						
						pathAllowDiagonals = EditorGUILayout.Toggle(new GUIContent(TidyMessages.MAP_CREATOR_PATH_ALLOW_DIAGONALS,
					                                                    null,
					                                                    TidyTooltips.MAP_CREATOR_PATH_ALLOW_DIAGONALS
					                                                    ),pathAllowDiagonals);
						
					
						EditorGUILayout.BeginHorizontal();
					
						GUILayout.FlexibleSpace();
					
						if(GUILayout.Button(TidyMessages.MAP_CREATOR_DRAW_PATH,GUILayout.Height(30.0f),GUILayout.Width(200.0f))){
							DrawPath();
						}
					
						GUILayout.FlexibleSpace();
					
						EditorGUILayout.EndHorizontal();
					
						EditorGUI.indentLevel = 0;
					}
					break;
				
				}
			}
			
		}
		
		void DrawAdvancedOptions(){
									
			EditorGUI.indentLevel = 2;
						
			EditorGUILayout.BeginHorizontal();
			
			if(GUILayout.Button(new GUIContent(TidyMessages.MAP_CREATOR_HELP,TidyMessages.MAP_CREATOR_HELP_TOOLTIP))){
				GoToURL(TidyFileNames.HELP_URL);
			}
			
			EditorGUILayout.EndHorizontal();
									
			EditorGUILayout.BeginHorizontal();
			
			if(GUILayout.Button(new GUIContent(TidyMessages.MAP_CREATOR_ABOUT,TidyMessages.MAP_CREATOR_ABOUT_TOOLTIP))){
				GoToURL(TidyFileNames.ABOUT_URL);
			}
			
			EditorGUILayout.EndHorizontal();
			
			EditorGUI.indentLevel = 0;
			
		}		
		
		void GoToURL(string url){
			
			Application.OpenURL(url);
			
		}
		
		void DrawHorizontalLine(){
			
			if(lineTexture == null){
				GUILayout.Space(10.0f);
				return;
			}
			
			GUILayout.Label("",GUILayout.Width(parentWindow.position.width),GUILayout.Height(2.0f));
			
			Rect lineRect = GUILayoutUtility.GetLastRect();
						
			GUI.DrawTexture(lineRect,lineTexture);
			
			
		}
		
		void CreateMap(string mapName){
			
			GameObject newMap = new GameObject(mapName);
			
			//Let's align it to face the camera
			if(Camera.main != null){
				
				newMap.transform.position = Camera.main.transform.position + (Camera.main.transform.forward * 10.0f);
				
				newMap.transform.rotation = Quaternion.Euler(new Vector3(0.0f,-Camera.main.transform.rotation.y,0.0f));
				
			}
							
			BlockMap map = newMap.AddComponent<BlockMap>();
			
			map.chunkWidth = chunkWidth;
			map.chunkHeight = chunkHeight;
			map.tileScale = new Vector3(tileWidth,tileHeight,tileDepth);
			map.growthAxis = growthAxis;
			map.editorMap = true;
			
			map.Editor_InitializeMap(mapName,TidyEditorUtility.GetMapChunkPrefab());
			
			Camera c = Camera.main;
		
			if(c != null){
				
				int layerMask = 1 << 1;
			  	layerMask = ~layerMask;
				
				c.cullingMask = layerMask;
				
				//For new users, mapchunks won't refresh because they aren't prefabs
				//So we'll need to manually set this
				//Should be a trifle
				MapChunk[] m = GameObject.FindObjectsOfType(typeof(MapChunk)) as MapChunk[];
				
				for(int i = 0; i < m.Length; i++){
					m[i].gameObject.layer = 1;
				}
				
				if(growthAxis == BlockMap.GrowthAxis.Forward){
					
					c.transform.position = newMap.transform.position;
					
					Vector3 pos = c.transform.position;
					
					pos.y += chunkWidth * tileDepth;
					
					c.transform.position = pos;
					
					c.transform.forward = Vector3.down;
					
				}
				else{
					
					c.transform.position = newMap.transform.position;
					
					Vector3 pos = c.transform.position;
					
					pos.z -= chunkWidth * tileWidth;
					
					c.transform.position = pos;
					
					c.transform.forward = newMap.transform.forward * -1;
					
				}
			}
			
		}

		public void DrawScene(){
		}
		
		bool leftMouseDown = false;
		bool rightMouseDown = false;
		
		bool drawingPath = false;
		
		List<Block> pathBlocks = new List<Block>();
		
		public int pathWidth = 1;
		public bool pathRandomisation = false;
		public bool pathAllowDiagonals = false;
		
		void DrawPath(){
			
			drawingPath = false;
			
			
			if(pathBlocks.Count >= 2){
				
				List<Vector2> pathCoords = new List<Vector2>();
				
				for(int i = 0; i < pathBlocks.Count; i++){
							
					pathCoords.Add(new Vector3(pathBlocks[i].x,pathBlocks[i].y));
					
				}
				
				BlockMap pathMap = pathBlocks[0].blockMap;
				int depth = pathBlocks[0].depth;
				
				for(int i = 0; i < pathCoords.Count-1; i++){
						
					DrawPath(pathCoords[i],pathCoords[i+1], pathMap, depth);
					
				}
				
				SetEntireMapDirty(pathMap);	
				
			}
			
			pathBlocks.Clear();
			
		}
		
		void DrawPath(Vector2 a, Vector2 b, BlockMap pathMap, int depth){
			
			List<PathNode> path = PathFinding.GetPath((int)a.x,(int)a.y,(int)b.x,(int)b.y,pathMap,depth,pathAllowDiagonals,pathRandomisation,1000,true);
			
			if(path != null){
				
				BlockMap bm = pathMap;
								
				Block ba = bm.GetBlockAt((int)a.x,(int)a.y,depth);
				Block bb = bm.GetBlockAt((int)b.x,(int)b.y,depth);
				
				if(ba == null){
					Debug.LogWarning("Null path block at: " + a.x + "," + a.y + " - aborting.");
					return;
				}
				
				if(bb == null){
					Debug.LogWarning("Null path block at: " + b.x + "," + b.y + " - aborting.");
					return;
				}
				
				//set the start and end nodes, as they don't come back with the path
				//PaintBlock(ba, false);
				//PaintBlock(bb, false);
				
				
				int upper_val, lower_val;
				
				upper_val = (int)(pathWidth / 2.0f);
				lower_val = upper_val;
				
				if(pathWidth % 2 != 0){
					upper_val += 1;
				}
				
				
				for(int x = ba.x-lower_val; x < ba.x+upper_val; x++){
						
					for(int y = ba.y-lower_val; y < ba.y+upper_val; y++){
				
						Block pB = bm.GetBlockAt(x,y,depth);
						
						if(pB != null){
							PaintBlock(pB,false);
						}
						
					}	
					
				}
				
				for(int x = bb.x-lower_val; x < bb.x+upper_val; x++){
						
					for(int y = bb.y-lower_val; y < bb.y+upper_val; y++){
				
						Block pB = bm.GetBlockAt(x,y,depth);
						
						if(pB != null){
							PaintBlock(pB,false);
						}
						
					}	
					
				}
				
				for(int i = 0; i < path.Count; i++){
															
					for(int x = path[i].x-lower_val; x < path[i].x+upper_val; x++){
						
						for(int y = path[i].y-lower_val; y < path[i].y+upper_val; y++){
					
							Block pB = bm.GetBlockAt(x,y,depth);
							
							if(pB != null){
								PaintBlock(pB,false);
							}
							
						}	
						
					}
				}
				
			}
			else{
				Debug.LogWarning("No path could be found between " + a.ToString() + " and " + b.ToString());
			}
			
		}
		
		void RemoveFromPath(Block block){
			
			if(pathBlocks.Count == 0){
				return;
			}
			
			int depth = pathBlocks[0].depth;
			BlockMap pathMap = pathBlocks[0].blockMap;
			
			if(block.depth != depth){
				return;
			}
			
			if(block.blockMap != pathMap){
				return;
			}
						
			for(int i = 0; i < pathBlocks.Count; i++){
				Block c = pathBlocks[i];
				
				if(c.x == block.x && c.y == block.y){
					pathBlocks.RemoveAt(i);
					return;
				}
			}
			
			
		}
		
		bool CanAddToPath(Block block){
			
			if(pathBlocks.Count == 0){
				return true;
			}
			
			int depth = pathBlocks[0].depth;
			BlockMap pathMap = pathBlocks[0].blockMap;
			
			if(block.depth != depth){
				return false;
			}
			
			for(int i = 0; i < pathBlocks.Count; i++){
				Block c = pathBlocks[i];
				
				if(c.x == block.x && c.y == block.y){
					return false;
				}
			}
			
			if(block.blockMap != pathMap){
				return false;
			}
			
			return true;
		}
		
		public BlockMap GetFocusMap(){
			return this.focusMap;
		}
		
		void DrawFunctionalOverlays(BlockMap map){
						
			TidyFunctionalOverlay overlay = map.functionalOverlay;
			
			Color hc = Handles.color;
			Handles.color = Color.green;
						
			float radius = map.tileScale.y * 0.4f;
			
			for(int i = 0; i < overlay.mapData.Count; i++){
				
				if(overlay.mapData[i] == null || overlay.mapData[i].parentBlock == null){
					overlay.mapData.RemoveAt(i);
					i--;
					continue;
				}
				
				Handles.DrawWireDisc(overlay.mapData[i].parentBlock.transform.position,overlay.mapData[i].parentBlock.transform.forward,radius);
								
			}
			
			Handles.color = hc;
			
		}
		
		Block functionBlock = null;
		TidyFunctionalObject functionData = null;
		SerializedObject functionSObj = null;
		Vector2 fScrollPos = Vector2.zero;
		SerializedProperty prop = null;
		
		void SetFunctionBlock(Block functionBlock){
			this.functionBlock = functionBlock;
			
			if(functionBlock.blockMap.functionalOverlay == null){
				functionBlock.blockMap.AddFunctionalOverlay(false);
				SetEntireMapDirty(functionBlock.blockMap);
			}
			
			this.functionData = functionBlock.blockMap.functionalOverlay.GetDataAt(this.functionBlock.x,this.functionBlock.y,this.functionBlock.depth);
			if(this.functionData != null){
				this.functionSObj = new SerializedObject(this.functionData);
				this.prop = this.functionSObj.GetIterator();
			}
		}
		
		bool setTidyTarget = false;
		//string tidyTargetName = "";
		TidyTarget currentTidyTarget = null;
		
		void DrawFunctionBlockUI(){
						
			Color c = Handles.color;
			Handles.color = Color.yellow;
			
			float radius = functionBlock.blockMap.tileScale.y * 0.4f;
			
			Handles.DrawWireDisc(functionBlock.transform.position,functionBlock.transform.forward,radius);
			
			Handles.color = c;
			
			Vector2 pos = HandleUtility.WorldToGUIPoint(functionBlock.transform.position);
			
			Handles.BeginGUI();
			
			Rect r = new Rect(pos.x - 150.0f,pos.y + 25.0f,300.0f,175.0f);
			
			//GUI.skin = TidyEditorUtility.GetEditorGUISkin();
			
			//GUI.Box(r,"");
			
			//GUI.skin = null;
			
			GUI.DrawTexture(r,GetIcon(TidyFileNames.AREA_BACKGROUND));
			
			GUILayout.BeginArea(r);
			
			GUILayout.BeginVertical();
			
			GUILayout.Space(10.0f);
			
			if(functionData == null){
				//draw the 'add data' UI
				
				
			
				//draw the close UI
				GUILayout.BeginHorizontal();
				
				GUILayout.Label("Add object:");
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button("X")){
					SetEntireMapDirty(functionBlock.blockMap);
					functionBlock = null;
					functionData = null;
					prop = null;
				}
				
				GUILayout.EndHorizontal();
				
				if(functionalComponents != null && functionalComponents.Length > 0){
					for(int i = 0; i < functionalComponents.Length; i++){
						
						if(GUILayout.Button(functionalComponents[i].name)){
						
							TidyFunctionalObject o = UnityEngineInternal.APIUpdaterRuntimeServices.AddComponent(functionBlock.blockMap.gameObject, "Assets/Tidy Tile Mapper/Editor/Editor Logic/TidyMapCreator.cs (1502,33)", functionalComponents[i].name) as TidyFunctionalObject;
							
							o.SetParentBlock(functionBlock);
														
							functionBlock.blockMap.functionalOverlay.AddData(o);
							
							SetEntireMapDirty(functionBlock.blockMap);
							
							SetFunctionBlock(functionBlock);
														
						}
						
					}
				}
				else{
					GUILayout.Label("None");
				}
				
				
			}
			else{
				
				GUILayout.Label(functionData.parentBlock.name + " at " +"["+functionData.parentBlock.x + "," + functionData.parentBlock.y + "," + functionData.parentBlock.depth+"]");
				
				//draw the close UI
				GUILayout.BeginHorizontal();
				
				GUILayout.Label(functionData.GetType().Name);
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button("X")){
					SetEntireMapDirty(functionBlock.blockMap);
					functionBlock = null;
					functionData = null;
					prop= null;
				}
				
				GUILayout.EndHorizontal();
				
				GUILayout.Space(10.0f);
				
				fScrollPos = GUILayout.BeginScrollView(fScrollPos);
								
				try{
							
					prop = functionSObj.GetIterator();
					
					if(prop != null && functionSObj != null && functionData != null){
												
						prop.NextVisible(true);
											
						bool searchDeep = true;
						
						do{
														
							searchDeep = true;
							
							if(prop.propertyPath == "m_Script"){
								continue;
							}
							
							if(prop.name == "parentBlock"){
								continue;
							}
							
							if(prop.type == "Vector3f"){
								searchDeep = false;
							}
							
							if(prop.type == "Vector3f" && prop.name == "targetDifference" && currentTidyTarget != null){
								
								prop.vector3Value = currentTidyTarget.targetDifference;
								
								currentTidyTarget = null;
							}
							
							if(prop.name == "TidyTarget_parentBlock" && currentTidyTarget != null){
								
								prop.objectReferenceValue = currentTidyTarget.TidyTarget_parentBlock;
								
							}
							
							if(prop.type == "TidyTarget"){
																	
								EditorGUILayout.BeginHorizontal();
							
								GUILayout.Label(prop.name + ":");
								
								GUILayout.FlexibleSpace();
								
								Color gc = GUI.color;
								
								if(setTidyTarget == true){
									GUI.color = Color.green;
								}
								
								if(GUILayout.Button("Set")){
									setTidyTarget = true;
									//tidyTargetName = prop.name;
									currentTidyTarget = null;
								}
								
								GUI.color = gc;
								
								EditorGUILayout.EndHorizontal();
								
								continue;
							}
						
							EditorGUILayout.BeginHorizontal();
							EditorGUILayout.PropertyField(prop);
							
							EditorGUILayout.EndHorizontal();
							
						}while(prop.NextVisible(searchDeep));
						
						prop.Reset();
						
						functionSObj.ApplyModifiedProperties();	
						
						functionSObj.Update();
					}
					else{
						GUILayout.Label("Data null.");
					}
					
				}catch(Exception e){Debug.LogWarning(e.ToString());}
					
				GUILayout.EndScrollView();
				
				GUILayout.BeginHorizontal();
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button("Delete")){
					
					TidyFunctionalObject o = functionBlock.blockMap.functionalOverlay.RemoveDataAt(functionBlock.x,functionBlock.y,functionBlock.depth);
					
					GameObject.DestroyImmediate(o);
					
					functionData = null;
					prop = null;
										
					SetEntireMapDirty(functionBlock.blockMap);
					
					functionBlock = null;
				}
				
				GUILayout.EndHorizontal();
			}
						
			GUILayout.EndVertical();
				
			GUILayout.EndArea();
			
			Handles.EndGUI();
		}
		
		MonoScript[] functionalComponents;
		
		void RefreshFunctionalObjects(){
			functionalComponents = TidyEditorUtility.GetAllFunctionalObjects();			
		}
		
		public void DrawScene (SceneView sceneView)
		{			
			//Do all of our casting now
			CheckMouseOver();
						
			if(currentBehaviour != MapPaintBehaviour.Disabled && currentBehaviour != MapPaintBehaviour.Block_Move && currentBehaviour != MapPaintBehaviour.Plugin_Active){
				int controlID = GUIUtility.GetControlID(FocusType.Passive);
					
				Tools.current = Tool.None;
			
				if(Event.current.type == EventType.Layout){
					HandleUtility.AddDefaultControl(controlID);
				}
			}
			
			if(currentBehaviour == MapPaintBehaviour.Block_Move){
				
				Tools.current = Tool.Move;
				
			}
			
			if(currentBehaviour == MapPaintBehaviour.Edit_Functions){
				
				for(int i = 0; i < existentBlockMaps.Length; i++){
					
					if(existentBlockMaps[i].functionalOverlay == null){
						
						continue;
					}
					
					DrawFunctionalOverlays(existentBlockMaps[i]);
					
				}
				
				
				if(functionBlock == null || setTidyTarget){
					
					Block b = mouseoverBlock;
					
					if(b != null){
						
						Color c = Handles.color;
					
						Handles.color = Color.yellow;
						
						float radius = b.blockMap.tileScale.y * 0.4f;
						
						Handles.DrawWireDisc(b.gameObject.transform.position,b.gameObject.transform.forward,radius);
						
						Handles.color = c;
						
					}
				}
				
			}
			
			if(drawingPath && pathBlocks.Count > 0){
				
				BlockMap pathMap = pathBlocks[0].blockMap;
				
				float radius = pathMap.tileScale.y * 0.4f;
				
				Color c = Handles.color;
				Handles.color = Color.green;
				
				for(int i = 0; i < pathBlocks.Count; i++){
					
					Handles.DrawWireDisc(pathBlocks[i].transform.position,pathBlocks[i].transform.forward,radius);
					
					if(i > 0){
						
						Handles.DrawLine(pathBlocks[i].transform.position,pathBlocks[i-1].transform.position);
						
					}
					
				}
				
				Handles.color = c;
				
			}
			
			//Debug.Log("Handle utility repaint");
			
			//HandleUtility.Repaint();
			
			Handles.BeginGUI();
						
			GUILayout.BeginArea(new Rect(0.0f,0.0f,Screen.width,Screen.height));
			
			GUILayout.BeginVertical();
						
			if(currentBehaviour == MapPaintBehaviour.Disabled){
				
				Color c = GUI.color;
				
				GUI.color = Color.white;
				
				GUILayout.Label(TidyMessages.MAP_CREATOR_INACTIVE);
									
				GUI.color = c;
												
			}
			else{
			
				Color c = GUI.color;
			
				GUI.color = Color.green;
				
				if(currentBehaviour == MapPaintBehaviour.Cycle){
					GUILayout.Label(TidyMessages.MAP_CREATOR_CYCLING_ACTIVE);
				}
				else
				if(currentBehaviour == MapPaintBehaviour.Paint){
					GUILayout.Label(TidyMessages.MAP_CREATOR_PAINTING_ACTIVE);
				}
				else
				if(currentBehaviour == MapPaintBehaviour.Block_Move){
					GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_MOVE_ACTIVE);
				}		
				
				GUI.color = c;
				
				OutputMouseoverState();
			}
			
						
			GUILayout.EndVertical();
									
			GUILayout.EndArea();
			
			Handles.EndGUI();
			
			if(currentBehaviour == MapPaintBehaviour.Block_Move){
				
					if(selectedBlock != null && focusMap != null){
						
						if(selectedBlock.transform.localPosition != selectedBlockPosition){
					
							if(canAct){
						
								HandleBlockMove(selectedBlock,selectedBlockPosition);
								selectedBlockPosition = selectedBlock.transform.localPosition;
							
								HasActed();
							
							}
						}
						
					}
				
				
			}
			
			bool initializedChunk = false;
					
			if(currentBehaviour != MapPaintBehaviour.Disabled && currentBehaviour != MapPaintBehaviour.Block_Move){
											
				if(Event.current.type == EventType.MouseUp || Event.current.type == EventType.mouseUp){
					
					leftMouseDown = false;
					rightMouseDown = false;
					
				}
				
				
				//these things only occur on mousedown
				if(Event.current.type == EventType.mouseDown || Event.current.type == EventType.MouseDown && (SceneView.currentDrawingSceneView.position.Contains(Event.current.mousePosition))){
				
					//only on left click
					if(Event.current.button == 0){
								
						rightMouseDown = false;
						
						leftMouseDown = true;
												
						if(currentBehaviour == MapPaintBehaviour.DrawPath){
							
							if(mouseoverBlock != null){
							
								if(CanAddToPath(mouseoverBlock)){
									pathBlocks.Add(mouseoverBlock);
									drawingPath = true;
								}
								
							}
							
						}
						else
						if(currentBehaviour == MapPaintBehaviour.Paint || currentBehaviour == MapPaintBehaviour.Cycle){
							
							//we are changing a chunk!
							if(mouseoverChunk != null){
								BlockMap map = mouseoverChunk.GetParentMap();
								
								focusMap = map;
								
								OrientedBlock[] defaultBlocks = new OrientedBlock[map.chunkWidth*map.chunkHeight];
								
								bool isEmptyBlock = TidyEditorUtility.IsEmptyBlock(workingBlock);
								
								TriggerSelection();
								
								for(int i = 0; i < defaultBlocks.Length; i++){
									
									//defaultBlocks[i] = EditorUtility.InstantiatePrefab(workingBlock) as OrientedBlock;
									
									if(isEmptyBlock){
										
										defaultBlocks[i] = GameObject.Instantiate(workingBlock) as OrientedBlock;
										
										defaultBlocks[i].name = workingBlock.name;
										
										
										if(map.growthAxis == BlockMap.GrowthAxis.Up){
										
											defaultBlocks[i].transform.localScale = new Vector3(map.tileScale.x,map.tileScale.y,map.tileScale.z);
											
										}
										else if(map.growthAxis == BlockMap.GrowthAxis.Forward){
											
											defaultBlocks[i].transform.localScale = new Vector3(map.tileScale.x,map.tileScale.z,map.tileScale.y);
											
										}
										
									}
									else{
										
										defaultBlocks[i] = PrefabUtility.InstantiatePrefab(workingBlock) as OrientedBlock;
										
										defaultBlocks[i].name = workingBlock.name;
																				
										BoxCollider b = defaultBlocks[i].GetComponent<BoxCollider>();
										
										
										if(map.growthAxis == BlockMap.GrowthAxis.Up){
											b.size = new Vector3(map.tileScale.x,map.tileScale.y,map.tileScale.z);
										}
										else{
											b.size = new Vector3(map.tileScale.x,map.tileScale.z,map.tileScale.y);
										}
									}
								}
								
								int depth = mouseoverChunk.depth;
								
								//MapChunk mc = map.GetChunkAt(mouseoverChunk.GetX(),mouseoverChunk.GetY(),depth,false);
								
								map.Editor_InitializeChunkAt(mouseoverChunk.GetX(),mouseoverChunk.GetY(),depth,defaultBlocks,TidyEditorUtility.GetMapChunkPrefab());
								
								HasActed();
								
								initializedChunk = true;
								
								//and then refresh the blocks around it
																
								int m_x = mouseoverChunk.GetX();
								int m_y = mouseoverChunk.GetY();
								
								//MapChunk adjacentChunk = null;
								
								for(int x = m_x-1; x <= m_x+1; x++){
									for(int y = m_y-1; y <= m_y+1; y++){
																			
										MapChunk m = mouseoverChunk.parentMap.GetChunkAt(x,y,depth,false);
										
										if(m != null && m.Editor_IsInitialized()){
											
											//Debug.Log("Set chunk dirty at: " + x + ", " + y);
											
											m.RefreshChunk();
											
											//for(int i = 0; i < m.chunkPieces.Length; i++){
												//EditorUtility.SetDirty(m.chunkPieces[i]);
											//}
											
											//EditorUtility.SetDirty(m);
											
											/*if(!(x == m_x && y == m_y)){
												
												if(x == m_x || y == m_y){
													
													adjacentChunk = m;
												}
											}*/
											
										}
									}
								}
								
								/*if(backgroundMaterial != null && (workingBlock.isNullBlock || workingBlock.actAsEmptyBlock)){
																	
									//We have to do this from a populated area to an unpopulated are
									
									int x_lower = 0;
									int x_upper = mouseoverChunk.parentMap.chunkWidth;
									int x_direction = 1;
									int y_lower = 0;
									int y_upper = mouseoverChunk.parentMap.chunkHeight;
									int y_direction = 1;
									
									if(adjacentChunk != null){
										
										int x_dif = m_x - adjacentChunk.GetX();
										int y_dif = m_y - adjacentChunk.GetY();
										
										if(x_dif == -1){
											x_lower = mouseoverChunk.parentMap.chunkWidth-1;
											x_upper = -1;
											x_direction = -1;
										}
										
										if(y_dif == -1){
											y_lower = mouseoverChunk.parentMap.chunkHeight-1;
											y_upper = -1;
											y_direction = -1;
										}
										
									}
									
									List<Vector3> coord = new List<Vector3>();
									
									for(int x = x_lower; x != x_upper; x+=x_direction){
										for(int y = y_lower; y != y_upper; y+=y_direction){
																						
											Block b = mouseoverChunk.GetBlockAtChunkCoord(x,y);
											
											coord.Add(new Vector3(b.x,b.y,b.depth));
											
										}
									}
									
									if(!mouseoverChunk.parentMap.HasBackgroundEntryFor(backgroundMaterial.name)){
																				
										mouseoverChunk.parentMap.AddBackground(backgroundMaterial);
									}
									
									mouseoverChunk.parentMap.AddToBackground(coord.ToArray(),backgroundMaterial.name);
								}*/
								
								SetEntireMapDirty(mouseoverChunk.parentMap);
																
							}
							
						}
						
						if(mouseoverBlock != null && !initializedChunk){
				
							if(currentBehaviour == MapPaintBehaviour.Cycle){
																			
								CycleBlock(mouseoverBlock);
								
							}
							
						}
						
						if(currentBehaviour == MapPaintBehaviour.Edit_Functions && functionBlock != null){
						}
						else{
							Event.current.Use();
						}
						
						
					}
					
					//on right-click
					if(Event.current.button == 1){
						
						rightMouseDown = true;
						
						leftMouseDown = false;
						
						if(mouseoverBlock != null){
											
							if(currentBehaviour == MapPaintBehaviour.Cycle){
																			
								CycleBlockVariation(mouseoverBlock,1);
								
							}
							
						}
						
												
						if(currentBehaviour == MapPaintBehaviour.DrawPath){
							
							if(mouseoverBlock != null){
							
								RemoveFromPath(mouseoverBlock);
								
								if(pathBlocks.Count == 0){
									drawingPath = false;
								}
								
							}
							
						}
						
					}
					
					//On middle mouse
					if(Event.current.button == 2){
						
						rightMouseDown = false;
						
						leftMouseDown = false;
						
						if(mouseoverBlock != null){
				
							if(currentBehaviour == MapPaintBehaviour.Cycle){
																			
								CycleBlockVariation(mouseoverBlock,-1);
								
							}
							
						}
						
					}
				
				}
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Edit_Functions){
				leftMouseDown = false;
				
				if(functionBlock == null && mouseoverBlock != null){
					SetFunctionBlock(mouseoverBlock);
				}
				
				if(setTidyTarget && mouseoverBlock != null && functionBlock != null){
					setTidyTarget = false;
					currentTidyTarget = new TidyTarget(functionBlock,mouseoverBlock);
				}
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Paint_Background){
				
				if(mouseoverBlock != null){
					AddToBackground(mouseoverBlock);			
				}
			}
			
			if(functionBlock != null){
				
				//Draw the function block UI
				
				DrawFunctionBlockUI();
				
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Delete_Chunk && mouseoverChunk != null){
				DeleteChunk(mouseoverChunk);
				leftMouseDown = false;
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Paint && mouseoverBlock != null  && !initializedChunk){
				
				PaintBlock(mouseoverBlock);
				
			}
			
			if(rightMouseDown && currentBehaviour == MapPaintBehaviour.Paint && mouseoverBlock != null  && !initializedChunk){
				
				//Nope: As this overrides the ability to move the camera around with right click
				
				//PaintEmptyBlock(mouseoverBlock);
				
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Add_Layer_Above && (mouseoverChunk != null || mouseoverBlock != null)){
				
				
				MapChunk c = mouseoverChunk;
				
				if(c == null){
					
					c = mouseoverBlock.blockMap.GetChunkForBlockCoordinate(mouseoverBlock.x,mouseoverBlock.y,mouseoverBlock.depth);
					
				}
				
				AddLayerChunk(c,1);
				
				parentWindow.Repaint();
				
				leftMouseDown = false;
				
			}
			
			if(leftMouseDown && currentBehaviour == MapPaintBehaviour.Add_Layer_Below && (mouseoverChunk != null || mouseoverBlock != null)){
				
				
				MapChunk c = mouseoverChunk;
				
				if(c == null){
					
					c = mouseoverBlock.blockMap.GetChunkForBlockCoordinate(mouseoverBlock.x,mouseoverBlock.y,mouseoverBlock.depth);
					
				}
				
				AddLayerChunk(c,-1);
				
				parentWindow.Repaint();
				
				leftMouseDown = false;
				
			}
			
			/*for(int i = 0; i < plugins.Length; i++){
				
				if(plugins[i].obj == null){
					continue;
				}
				
				plugins[i].obj.DrawScene(sceneView);
			}*/
		}
		
		void DeleteChunk(MapChunk targetChunk){
			
			if(!targetChunk.Editor_IsInitialized()){
				
				targetChunk.parentMap.RemoveChunkAt(targetChunk.x,targetChunk.y, targetChunk.depth);
				
				SetEntireMapDirty(targetChunk.parentMap);
				
				Editor.DestroyImmediate(targetChunk.gameObject);
			}
			
		}
		
		void AddLayerChunk(MapChunk chunk, int direction){
			
			if(!canAct){
				return;
			}
			
			HasActed();
			
			//Debug.Log(chunk);
			
			int depth = chunk.depth + direction;
			
			//Debug.Log("Adding chunk at: " + depth);
			
			chunk.parentMap.Editor_AddChunkAt(chunk.x,chunk.y,depth,TidyEditorUtility.GetMapChunkPrefab(),false);
			
			MapChunk c = chunk.parentMap.GetChunkAt(chunk.x,chunk.y,depth,false);
			
			EditorUtility.SetDirty(c);
			EditorUtility.SetDirty(chunk.parentMap);
			
			RefreshExistentBlockMaps();
			
		}
		
		void DrawMapInfo(){
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_CURRENT_MAP_CATEGORY,EditorStyles.boldLabel);
			
			GUILayout.Space(10.0f);
			
			if(currentBehaviour != MapPaintBehaviour.Disabled){
				
				GUI.enabled = false;
				
			}
			
				
			string mapName = TidyMessages.MAP_CREATOR_NO_MAP_SELECTED;
			int width = 0, height = 0;
			
			if(focusMap != null){
				mapName = focusMap.name;
				width = focusMap.currentWidth;
				height = focusMap.currentHeight;
			}
			
							
			GUILayout.Label(TidyMessages.MAP_CREATOR_SELECTED_MAP_LABEL + mapName);
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_CHUNK_WIDTH + width + TidyMessages.MAP_CREATOR_CHUNK_WIDTH_SUFFIX);
			GUILayout.Label(TidyMessages.MAP_CREATOR_CHUNK_HEIGHT + height + TidyMessages.MAP_CREATOR_CHUNK_HEIGHT_SUFFIX);
		
			if(focusMap == null){
				GUI.enabled = false;
			}
			
			selectedStrippingLevel = (StrippingLevel)EditorGUILayout.EnumPopup(TidyMessages.MAP_CREATOR_STRIPPING_LEVEL,selectedStrippingLevel);
			
			EditorGUILayout.BeginHorizontal();
			
			if(GUILayout.Button("Refresh")){
				RefreshAllMaps();
				SetEntireMapDirty(focusMap);
			}
			
			GUILayout.FlexibleSpace();
			
			if(GUILayout.Button(new GUIContent(TidyMessages.MAP_CREATOR_PUBLISH_MAP,TidyTooltips.MAP_CREATOR_PUBLISH_MAP))){
				PrePublishMap(focusMap);
			}
			
			
			GUILayout.Space(20.0f);
			
			EditorGUILayout.EndHorizontal();
			
			if(focusMap == null){
				GUI.enabled = true;
			}
			GUILayout.Label(mapPublishStatus,italicStyle);
			
			if(currentBehaviour != MapPaintBehaviour.Disabled){
				
				GUI.enabled = true;
				
			}
			
		}
		
		public Block GetMouseOverBlock(){
			return mouseoverBlock;
		}
		
		public MapChunk GetMouseOverChunk(){
			return mouseoverChunk;
		}
		
		Block mouseoverBlock;
		
		MapChunk mouseoverChunk;
		
		void CheckMouseOver(){
			
			Ray worldRay = HandleUtility.GUIPointToWorldRay (Event.current.mousePosition);
			
			mouseoverBlock = null;
			mouseoverChunk = null;
			
			//Note to the viewer: I don't like doing this, I would prefer to use layers
			//But in the absence of an API method for adding layers programmatically, this (to me)
			//is a preferable choice to adding a setup step:
			//Step One) add a layer named y
			RaycastHit[] hits = Physics.RaycastAll(worldRay);
			
			if(hits != null){
					
				float dist = float.MaxValue;
				
				MapChunk closest_mc = null;
				Block closest_b = null;
				
				for(int i = 0; i < hits.Length; i++){
					
					MapChunk mc = hits[i].collider.gameObject.GetComponent<MapChunk>();
					
					
					if(mc != null){
						
						if(!IsLayerUnlocked(mc)){
							continue;
						}
						
						if(hits[i].distance < dist){
							closest_mc = mc;
							closest_b = null;
							dist = hits[i].distance;
						}
						
						continue;
					}
						
					Block b = hits[i].collider.gameObject.GetComponent<Block>();
						
					if(b != null){
									
						if(!IsLayerUnlocked(b)){
							continue;
						}
						
						if(hits[i].distance < dist){
							closest_b = b;
							closest_mc = null;
							dist = hits[i].distance;
						}
					}
				}
				
				if(closest_mc != null){
					
					mouseoverChunk = closest_mc;
					
				}
				else
				if(closest_b != null){
					
					mouseoverBlock = closest_b;
					
				}
				
			}
						
		}
		
		void OutputMouseoverState(){
			
			string bName = TidyMessages.MAP_CREATOR_DEFAULT_BLOCK_NAME;
			string coords = TidyMessages.MAP_CREATOR_DEFAULT_BLOCK_COORDS;
			string orientation = TidyMessages.MAP_CREATOR_DEFAULT_BLOCK_ORIENTATION;
			
			if(mouseoverBlock != null){
				bName = mouseoverBlock.name;
				coords = mouseoverBlock.x + ", " + mouseoverBlock.y + ", " + mouseoverBlock.depth;
				
				if(mouseoverBlock is OrientedBlock){
					
					OrientedBlock ob = mouseoverBlock as OrientedBlock;
					
					orientation = ob.GetOrientation().ToString();
			
				}
			}
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_NAME_PREFIX + bName);
			GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_COORDS_PREFIX + coords);
			GUILayout.Label(TidyMessages.MAP_CREATOR_BLOCK_ORIENTATION_PREFIX + orientation);
							
		}
		
		public void Destroy ()
		{
			//Moved to public script
			//SceneView.onSceneGUIDelegate -= DrawScene;
		}
		#endregion
		
		public void SetCanAct(bool canAct){
			this.canAct = canAct;
		}
		
		public void PaintBlock(Block block, bool triggerAct){
			
			PaintBlock(block);
			
			if(!triggerAct){
				canAct = true;
			}
			
		}
		
		void PaintEmptyBlock(Block block){
			
			if(!canAct){
				return;
			}
						
			if(block.name == workingBlock.name){
				return;
			}
			
			TriggerSelection();
			
			int x = block.x;
			int y = block.y;
			
			Block newBlock = null;
			
			//newBlock = EditorUtility.InstantiatePrefab(workingBlock) as Block;
			
			newBlock = PrefabUtility.InstantiatePrefab(workingBlocks[0]) as Block;
			
			if(TidyEditorUtility.IsEmptyBlock(newBlock)){
									
				if(block.blockMap.growthAxis == BlockMap.GrowthAxis.Up){
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.y,block.blockMap.tileScale.z);
				}
				else{
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.z,block.blockMap.tileScale.y);
				}
				
			}
			
			block.blockMap.SetBlockAt(x,y,block.depth,newBlock,true);
						
			RefreshBlocks(x,y,newBlock.depth,newBlock.blockMap);
			
			EditorUtility.SetDirty(newBlock);
			
			MapChunk c = newBlock.blockMap.GetChunkForBlockCoordinate(newBlock.x,newBlock.y,newBlock.depth);
			
			if(c != null){
				EditorUtility.SetDirty(c);
			}
			
			HasActed();
			
		}
		
		//We need to populate these somehow
		public List<Material> backgroundMaterials = new List<Material>();
		public Material backgroundMaterial;
		
		Vector2 bgScrollPos;
		Rect bgScrollRect;
		
		string MATERIAL_KEY = "TTM_BG_Materials";
		char MATERIAL_SPLIT = '|';
		
		void LoadBackgroundMaterials(){
			//Hmm
			
			backgroundMaterials.Clear();
			
			backgroundMaterials.Add(null);
			
			if(EditorPrefs.HasKey(MATERIAL_KEY)){
				
				string mPaths = EditorPrefs.GetString(MATERIAL_KEY);
				string[] mPSplit = mPaths.Split(MATERIAL_SPLIT);
				
				for(int i = 0; i < mPSplit.Length; i++){
				
					Material m = AssetDatabase.LoadAssetAtPath(mPSplit[i],typeof(Material)) as Material;
					
					if(m != null){
						backgroundMaterials.Add(m);
					}
					
				}
			}
		}
		
		void RemoveBackgroundMaterial(Material m){
			string path = AssetDatabase.GetAssetPath(m);
			
			if(EditorPrefs.HasKey(MATERIAL_KEY)){
				
				string mPaths = EditorPrefs.GetString(MATERIAL_KEY);
				string[] mPSplit = mPaths.Split(MATERIAL_SPLIT);
				
				string nPath = "";
				
				for(int i = 0; i < mPSplit.Length; i++){
				
					if(mPSplit[i] == path){
						continue;
					}
					
					nPath += mPSplit + MATERIAL_KEY;
					
				}
				
				EditorPrefs.SetString(MATERIAL_KEY,nPath);
				
				LoadBackgroundMaterials();
			}
		}

		void AddBackgroundMaterial(Material m){
			//Double Hmm
			string path = AssetDatabase.GetAssetPath(m);
			
			if(EditorPrefs.HasKey(MATERIAL_KEY)){
				string mPaths = EditorPrefs.GetString(MATERIAL_KEY);
				mPaths += MATERIAL_SPLIT + path;
				EditorPrefs.SetString(MATERIAL_KEY,mPaths);
			}
			else{
				EditorPrefs.SetString(MATERIAL_KEY,path);
			}
				
			LoadBackgroundMaterials();
		}
		
		void DrawBackgroundPanel(){
			
			GUILayout.Space(10.0f);
			
			GUILayout.Label("Your Backgrounds");
						
			GUILayout.Space(10.0f);
			
			/*GUILayout.Label("Automated background");
			
			GUILayout.Space(10.0f);
			
			if(focusMap == null){
				GUILayout.Label("No map selected");
				return;
			}
			
			currentMapMaterial = EditorGUILayout.ObjectField("Background material:",currentMapMaterial,typeof(Material),false) as Material;
			
			if(currentMapMaterial == null){
				GUI.enabled = false;
			}
			
			if(focusMap.HasBackground()){
				
				if(GUILayout.Button("Set material")){
					focusMap.SetBackgroundMaterial(currentMapMaterial);
				}
			}
			else{
				if(GUILayout.Button("Add background")){
					focusMap.AddBackground(currentMapMaterial);
					SetEntireMapDirty(focusMap);
				}
			}
			
			focusMap.backgroundBehindAll = !EditorGUILayout.Toggle("Only for empty squares:",!focusMap.backgroundBehindAll);
			focusMap.backgroundDepth = EditorGUILayout.IntField("Background layer:",focusMap.backgroundDepth);
			focusMap.background_x_tiling = EditorGUILayout.FloatField("Background x tiling:",focusMap.background_x_tiling);
			focusMap.background_y_tiling = EditorGUILayout.FloatField("Background y tiling:",focusMap.background_y_tiling);
			
			if(currentMapMaterial == null){
				GUI.enabled = true;
			}
			
			if(focusMap.HasBackground()){
				
				if(GUILayout.Button("Remove Background")){
					focusMap.RemoveBackground(true);
				}
			}*/
			
			int rows = backgroundMaterials.Count / tilesAcross + 1;
			
			bgScrollPos = EditorGUILayout.BeginScrollView(bgScrollPos,GUILayout.Width(parentWindow.position.width),GUILayout.MinHeight(idealBlockIconWidth * rows + 10.0f));
			
			Rect bgScrollRect = EditorGUILayout.BeginVertical();
		
			GUI.Box(bgScrollRect,"");
			
			for(int i = 0; i < backgroundMaterials.Count; i++){
				
				if(backgroundMaterials[i] == null && i != 0){
					LoadBackgroundMaterials();
					continue;
				}
				
				if(i % tilesAcross == 0){
					
					if(i > 0){
						GUILayout.FlexibleSpace();
						EditorGUILayout.EndHorizontal();
					}
					
					EditorGUILayout.BeginHorizontal();
					
				}
				
				string name = "";
				Texture2D texture = null;
					
				if(i == 0){
					name = "None";
					texture = null;
				}
				else{
					name = backgroundMaterials[i].name;
					
					texture = backgroundMaterials[i].mainTexture as Texture2D;				
				}
				
				if(GUILayout.Button(new GUIContent("",name),GUILayout.Width(idealBlockIconWidth+extraBit),GUILayout.Height(idealBlockIconWidth+extraBit))){
					backgroundMaterial = backgroundMaterials[i];
				}
								
				if(texture != null){
					GUI.DrawTexture(GUILayoutUtility.GetLastRect(),texture,ScaleMode.ScaleAndCrop,false);
				}
				
				if(backgroundMaterials[i] == backgroundMaterial){
					GUI.DrawTexture(GUILayoutUtility.GetLastRect(),itemSelectedTexture,ScaleMode.ScaleToFit,true);	
				}
				
				if(i == backgroundMaterials.Count-1){
					GUILayout.FlexibleSpace();
					EditorGUILayout.EndHorizontal();
					
				}
				
			}
			
			EditorGUILayout.EndVertical();
						
			EditorGUILayout.EndScrollView();
			
			//Now an add and delete button
			
			EditorGUILayout.BeginHorizontal();
			
			
			if(backgroundMaterial == null){
				GUI.enabled = false;
			}
			
			if(GUILayout.Button("Remove")){
				RemoveBackgroundMaterial(backgroundMaterial);
			}
			
			if(backgroundMaterial == null){
				GUI.enabled = true;
			}
			
			GUILayout.FlexibleSpace();
					
			EditorGUILayout.EndHorizontal();
			
			addMaterial = EditorGUILayout.ObjectField("Background Material",addMaterial,typeof(Material),false) as Material;
			
			EditorGUILayout.BeginHorizontal();
			
			GUILayout.FlexibleSpace();
			
			if(addMaterial == null){
				GUI.enabled = false;
			}
			
			if(GUILayout.Button("Add Background Material")){
				AddBackgroundMaterial(addMaterial);
				addMaterial = null;
			}
			
			if(addMaterial == null){
				GUI.enabled = true;
			}
			
			GUILayout.Space(20.0f);
			
			EditorGUILayout.EndHorizontal();
			
			if(focusMap != null){
				
				focusMap.background_x_tiling = EditorGUILayout.FloatField("Tiling (X):",focusMap.background_x_tiling);
				focusMap.background_y_tiling = EditorGUILayout.FloatField("Tiling (Y):",focusMap.background_y_tiling);
				focusMap.backgroundDepth = EditorGUILayout.IntField("Background Layer:",focusMap.backgroundDepth);
				
				EditorGUILayout.BeginHorizontal();	
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button("Refresh backgrounds")){
					focusMap.RecalculateBackground();
				}
				
				GUILayout.Space(20.0f);
				
				EditorGUILayout.EndHorizontal();	
				
				EditorGUILayout.BeginHorizontal();	
				
				GUILayout.FlexibleSpace();
				
				if(GUILayout.Button("Delete backgrounds")){
					focusMap.RemoveBackground(true);
				}
				
				GUILayout.Space(20.0f);
				
				EditorGUILayout.EndHorizontal();	
			}
		}
		
		Material addMaterial = null;
				
		void AddToBackground(Block b){
			
			if(backgroundMaterial == null){
				
				BlockMap.BackgroundEntry e = b.blockMap.RemoveFromBackground(new Vector3(b.x,b.y,b.depth));
					
				if(e != null){
				
					if(e.IsEmpty()){
						b.blockMap.RemoveBackground(e,true);
					}
				}
				
				b.blockMap.RecalculateBackground();
				
				EditorUtility.SetDirty(b.blockMap.gameObject);
				EditorUtility.SetDirty(b.blockMap);
				
				return;
				
			}
			
			if(!b.blockMap.HasBackgroundEntryFor(backgroundMaterial.name)){
				b.blockMap.AddBackground(backgroundMaterial);
			}
			
			b.blockMap.AddToBackground(new Vector3(b.x,b.y,b.depth),backgroundMaterial.name);
			
			b.blockMap.RecalculateBackground();
			
			EditorUtility.SetDirty(b.blockMap.gameObject);
			EditorUtility.SetDirty(b.blockMap);
						
		}
		
		void PaintBlock(Block block){
		
			if(!canAct){
				return;
			}
			
			if(block.name == workingBlock.name && !paintRandom){
				return;
			}
						
			int x = block.x;
			int y = block.y;
			
			Block newBlock = null;
			
			//newBlock = EditorUtility.InstantiatePrefab(workingBlock) as Block;
			
			newBlock = PrefabUtility.InstantiatePrefab(workingBlock) as Block;
			
			if(TidyEditorUtility.IsEmptyBlock(newBlock)){
									
				if(block.blockMap.growthAxis == BlockMap.GrowthAxis.Up){
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.y,block.blockMap.tileScale.z);
				}
				else{
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.z,block.blockMap.tileScale.y);
				}
				
			}
			else{
				BoxCollider b = newBlock.GetComponent<BoxCollider>();
				
				
				if(block.blockMap.growthAxis == BlockMap.GrowthAxis.Up){
					b.size = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.y,block.blockMap.tileScale.z);
				}
				else{
					b.size = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.z,block.blockMap.tileScale.y);
				}
			}
			
			//if((block.isNullBlock || block.actAsEmptyBlock || block.blockMap.backgroundBehindAll) && block.depth == block.blockMap.backgroundDepth){
				//RemoveFromBackground(block);
			//}
			
			block.blockMap.SetBlockAt(x,y,block.depth,newBlock,true);
			
			if(paintRandom){
				
				newBlock.RandomiseVariant();
				
			}
			
			RefreshBlocks(x,y,newBlock.depth,newBlock.blockMap);
			
			//if((newBlock.isNullBlock || newBlock.actAsEmptyBlock || newBlock.blockMap.backgroundBehindAll) && newBlock.depth == newBlock.blockMap.backgroundDepth){
				//AddToBackground(newBlock);
			//}
			
			SetBlockDirty(newBlock);
			
			MapChunk c = newBlock.blockMap.GetChunkForBlockCoordinate(newBlock.x,newBlock.y,newBlock.depth);
			
			if(c != null){
				EditorUtility.SetDirty(c);
			}
			
			HasActed();
		}
		
		void SetBlockDirty(Block b){
			
			for(int i = 0; i < b.transform.childCount; i++){
				
				GameObject o = b.transform.GetChild(i).gameObject;
						
				EditorUtility.SetDirty(o);
				
			}
			
			EditorUtility.SetDirty(b);
			
		}
		
		void CycleBlock(Block block){
			
			TriggerSelection();
			
			int x = block.x;
			int y = block.y;
			
			int index = GetIndexForBlockName(block.name);
			
			index++;
			
			if(index >= workingBlocks.Count){
				index = 0;
			}
						
			//Block newBlock = EditorUtility.InstantiatePrefab(workingBlocks[index]) as Block;
			
			Block newBlock = PrefabUtility.InstantiatePrefab(workingBlocks[index]) as Block;
			newBlock.name = workingBlocks[index].name;
			
			if(TidyEditorUtility.IsEmptyBlock(newBlock)){
									
				if(block.blockMap.growthAxis == BlockMap.GrowthAxis.Up){
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.y,block.blockMap.tileScale.z);
				}
				else{
					newBlock.transform.localScale = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.z,block.blockMap.tileScale.y);
				}
				
			}
			else{
				BoxCollider b = newBlock.GetComponent<BoxCollider>();
				
				
				if(block.blockMap.growthAxis == BlockMap.GrowthAxis.Up){
					b.size = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.y,block.blockMap.tileScale.z);
				}
				else{
					b.size = new Vector3(block.blockMap.tileScale.x,block.blockMap.tileScale.z,block.blockMap.tileScale.y);
				}
			}
			
			if(block.blockMap == null){
				//Debug.LogWarning("Block: " + x + "," + y + " - " + block.name + " map is null.");
				return;
			}
			
			//if((block.isNullBlock || block.actAsEmptyBlock || block.blockMap.backgroundBehindAll) && block.depth == block.blockMap.backgroundDepth){
				//RemoveFromBackground(block);
			//}
			
			block.blockMap.SetBlockAt(x,y,block.depth,newBlock,true);
			
			RefreshBlocks(x,y,block.depth,newBlock.blockMap);
			
			//if((newBlock.isNullBlock || newBlock.actAsEmptyBlock || newBlock.blockMap.backgroundBehindAll) && newBlock.depth == newBlock.blockMap.backgroundDepth){
				//AddToBackground(newBlock);
			//}
			
			SetBlockDirty(newBlock);
			
			MapChunk c = newBlock.blockMap.GetChunkForBlockCoordinate(newBlock.x,newBlock.y,newBlock.depth);
			
			if(c != null){
				EditorUtility.SetDirty(c);
			}
		}
		
		void CycleBlockVariation(Block block, int direction){
			
			OrientedBlock b = block as OrientedBlock;
			
			b.CycleBlockVariation(direction);
			
			EditorUtility.SetDirty(b);
			
		}
		
		void RefreshBlocks(int x, int y, int depth, BlockMap map){
			
			EditorUtility.SetDirty(map);
			
			for(int x1 = x -1; x1 <= x + 1; x1++){
				for(int y1 = y - 1; y1 <= y + 1; y1++){
					
					Block b = map.GetBlockAt(x1,y1,depth);
					
					if(b != null){
						
						b.RefreshBlock();
						
						EditorUtility.SetDirty(b);
						
						//if(paintRandom){
							//b.RandomiseVariant();
						//}
						
					}
					
				}
			}
			
		}
		
		int GetIndexForBlockName(string name){
			
			for(int i = 0; i < workingBlocks.Count; i++){
				
				if(workingBlocks[i].name.Equals(name)){
					return i;
				}
				
			}
			
			return 0;
			
		}
		
						
		#region Texture management
		
		Texture2D GetPreviewForGameObject(GameObject gameObject, bool overwrite){
			
			if(previews == null){
				
				InitializePreviews();
				
			}
			
			return previews.GetPreviewTexture(gameObject,overwrite);
				
		}
		
		void InitializePreviews(){
			
			previews = new PreviewDictionary(TidyEditorUtility.previewPath);
			
		}
			
		//This is a sad but mandatory addition to the utility class
		//returns true if an update has occurred
		bool UpdatePreviews(){
			
			if(previews == null){
				
				InitializePreviews();
				
			}
			
			return previews.UpdatePreviewImports(Time.deltaTime);
			
		}
		
		Texture2D GetIcon(string iconName){
			
			if(icons == null){
				
				InitializeIcons();
				
			}
			
			return icons.GetTexture(iconName);
			
		}
		
		void InitializeIcons(){
			
			icons = new TextureDictionary(TidyEditorUtility.iconPath);
			
		}
	
		
		#endregion
		
		#region Map-specific handling
		
		BlockMap focusMap = null;
		
		Block selectedBlock = null;
		Vector3 selectedBlockPosition = Vector3.zero;
		
		bool selectionChangeRequired = false;
		
		public void OnSelectionChange(){
							
			if(Selection.activeGameObject != null){
				
				focusMap = GetBlockMapFromSelection(Selection.activeGameObject);
								
				selectedBlock = GetBlockFromSelection(Selection.activeGameObject);
				
				if(selectedBlock != null && Selection.activeGameObject != selectedBlock){
					selectedBlockPosition = selectedBlock.transform.localPosition;
					
					if(currentBehaviour == MapPaintBehaviour.Block_Move){
						selectionChangeRequired = true;
					}
				}
				
				//Debug.Log("Repaint selection");
				
				parentWindow.Repaint();
			}
		}
				
		public Block GetBlockFromSelection(GameObject o){
			
			if(o == null){
				return null;
			}
			
			Block b = o.GetComponent<Block>();
			
			if(b != null){
				
				return b;
				
			}
			
			if(o.transform.parent == null){
				return null;
			}
			
			return GetBlockFromSelection(o.transform.parent.gameObject);
			
		}
		
		public BlockMap GetBlockMapFromSelection(GameObject o){
			
			if(o == null){
				return null;
			}
			
			Block b = o.GetComponent<Block>();
			
			if(b != null){
				
				return b.blockMap;
				
			}
			
			BlockMap bm = o.GetComponent<BlockMap>();
			
			
			if(bm != null){
				
				return bm;
				
			}
			
			if(o.transform.parent == null){
				return null;
			}
			
			return GetBlockMapFromSelection(o.transform.parent.gameObject);
						
		}
		
		#endregion
	
		bool shouldPublish = false;
		bool hasUpdated = false;
		BlockMap publishMap = null;
		bool publishOverwrite = false;
		
		public void PrePublishMap(BlockMap map){
			
			if(map == null){
				return;
			}
			
			publishOverwrite = false;
			
			if(TidyEditorUtility.DoesMapPrefabExist(map.gameObject)){
				
				if(EditorUtility.DisplayDialog(TidyMessages.MAP_CREATOR_PUBLISH_MAP_EXISTS_TITLE,
				                               TidyMessages.MAP_CREATOR_PUBLISH_MAP_EXISTS_INFO,
				                               TidyMessages.MAP_CREATOR_PUBLISH_MAP_EXISTS_OVERWRITE,
				                               TidyMessages.MAP_CREATOR_PUBLISH_MAP_EXISTS_DONT_OVERWRITE
				                               )){
					//overwrite
					publishOverwrite = true;
				}
				else{
					//don't overwrite
					publishOverwrite = false;
				}
			}
			else{
				publishOverwrite = true;
			}
			
			publishMap = map;
			
			shouldPublish = true;
			hasUpdated = false;
			
			mapPublishStatus = TidyMessages.PUBLISH_UTILITY_BUILDING;
			
			parentWindow.ShowNotification(new GUIContent(TidyMessages.PUBLISH_UTILITY_NOTIFICATION));
			
			parentWindow.Repaint();
		}
		
		void PublishMap(){
			
			shouldPublish = false;
			hasUpdated = false;
			
			RefreshAllMaps();
			
			GameObject o = Editor.Instantiate(publishMap.gameObject) as GameObject;
			
			o.name = publishMap.gameObject.name;
			
			BlockMap newMap = o.GetComponent<BlockMap>();
			
			newMap.RefreshMap();
			
			newMap.hasBeenPublished = true;
			
			CleanMap(newMap);
			
			for(int i = 0; i < newMap.backgrounds.Count; i++){
			
				Mesh m = newMap.backgrounds[i].backgroundMeshFilter.sharedMesh;
				m.name = newMap.name + "_" + newMap.backgrounds[i].backgroundObject.name +  "_background";
				Mesh nm = TidyEditorUtility.SaveBackgroundMesh(m);
				newMap.SetBackgroundMesh(nm,newMap.backgrounds[i].backgroundMaterial.name);
			}
						
			mapPublishStatus = TidyEditorUtility.SaveMapAsPrefab(o,publishOverwrite);
						
			Editor.DestroyImmediate(o);
			
			parentWindow.RemoveNotification();
			
			parentWindow.Repaint();
			
		}
			
		void CleanMap(BlockMap blockMap){
			
			ChunkSet[] map = blockMap.map;
			
			//Strip empty blocks and such
			for(int i = 0; i < map.Length; i++){
				
				if(map[i] != null){
					
					for(int c = 0; c < map[i].chunkSet.Length; c++){
					
						MapChunk m = map[i].chunkSet[c].chunk;
						
						//Delete empty chunks
						if(m.chunkPieces == null || m.chunkPieces.Length <= 0){
							
							if(selectedStrippingLevel == StrippingLevel.Strip_All){
								Editor.DestroyImmediate(m.gameObject);
							}
							else{
								if(m.GetComponent<Renderer>() != null){
									Editor.DestroyImmediate(m.GetComponent<Renderer>());
								}
								
								MeshFilter mf = m.GetComponent<MeshFilter>();
								
								if(mf != null){
									Editor.DestroyImmediate(mf);
								}
								
								if(m.GetComponent<Collider>() != null){
									Editor.DestroyImmediate(m.GetComponent<Collider>());
								}
							}
							
							continue;
						}
						
						if(m.chunkPieces != null){
							
							for(int j = 0; j < m.chunkPieces.Length; j++){
							
								if(m.chunkPieces[j].isNullBlock){
									
									Block mb = m.chunkPieces[j];
									
									//Debug.Log("Hiding " + mb.x + "," + mb.y);
									mb.Hide();
									
									//Delete things below it on the hierarchy
									for(int d = 0; d < mb.transform.childCount; d++){
										
										GameObject o = mb.transform.GetChild(d).gameObject;
										
										Editor.DestroyImmediate(o);
										
									}
									
									if(selectedStrippingLevel == StrippingLevel.Working_Blocks_Only){
										m.chunkPieces[j].GetComponent<Collider>().isTrigger = true;
									}
									else{
										Editor.DestroyImmediate(m.chunkPieces[j].gameObject);
										continue;
									}
									
									continue;
								}
								else if(!m.chunkPieces[j].retainCollider){
									
									m.chunkPieces[j].GetComponent<Collider>().isTrigger = true;
									
								}
								
							}
							
						}
					}
					
				}
		
			
			}
			
			if(selectedStrippingLevel == StrippingLevel.Strip_All){
				
				OrientedBlock[] blocks = blockMap.GetComponentsInChildren<OrientedBlock>();
				MapChunk[] chunks = blockMap.GetComponentsInChildren<MapChunk>();
				
				for(int i = 0; i < blocks.Length; i++){
					Editor.DestroyImmediate(blocks[i]);
				}
				
				for(int i = 0; i < chunks.Length; i++){
					Editor.DestroyImmediate(chunks[i]);
				}
				
				Editor.DestroyImmediate(blockMap);
				
			}
		}
			
		public enum Block_Move_Selection{
			Square,
			Circle
		};
		
		Block_Move_Selection block_move_selection;
		int blockSelectionRadius = 1;
		float blockMoveRoundness = 0.0f;
		bool foldBlockMoveTools = true;
				
		void DrawMapTools(){
					
			Color c = GUI.color;
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_TOOL_SELECTION,EditorStyles.boldLabel);
						
			EditorGUILayout.BeginHorizontal();
			
			float btnWidth = (parentWindow.position.width - 10.0f) * 0.33f;
			
			if(btnWidth >= maximumButtonWidth){
				btnWidth = maximumButtonWidth;
			}
			
			
			if(currentBehaviour == MapPaintBehaviour.Paint){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ICON_PAINT),TidyTooltips.MAP_CREATOR_PAINT),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Paint);
			}
			
			GUI.color = c;
			
			if(currentBehaviour == MapPaintBehaviour.Cycle){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ICON_CYCLE),TidyTooltips.MAP_CREATOR_CYCLE),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Cycle);
			}
			
			GUI.color = c;
			
			if(focusMap == null || selectedBlock == null){
				GUI.enabled = false;
			}
			
			if(currentBehaviour == MapPaintBehaviour.Block_Move){
				GUI.color = Color.green;
			}
						
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ICON_MOVE_BLOCK),TidyTooltips.MAP_CREATOR_MOVE_BLOCK),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				
				SetPaintBehaviour(MapPaintBehaviour.Block_Move);
				
			}
			
			GUI.color = c;
			
			if(focusMap == null || selectedBlock == null){
				GUI.enabled = true;
			}
			
			
			if(currentBehaviour == MapPaintBehaviour.Disabled){
				GUI.color = Color.red;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ICON_DISABLE),TidyTooltips.MAP_CREATOR_DISABLE),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Disabled);
			}
			
			GUI.color = c;
			
			EditorGUILayout.EndHorizontal();
			
			EditorGUILayout.BeginHorizontal();
			
			if(currentBehaviour == MapPaintBehaviour.Add_Layer_Above){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ADD_LAYER_UP),TidyTooltips.MAP_CREATOR_ADD_LAYER_UP),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Add_Layer_Above);
			}
			
			GUI.color = c;
			
			if(focusMap == null || selectedBlock == null){
				GUI.enabled = true;
			}
			
			if(currentBehaviour == MapPaintBehaviour.Add_Layer_Below){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.ADD_LAYER_DOWN),TidyTooltips.MAP_CREATOR_ADD_LAYER_DOWN),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Add_Layer_Below);
			}
			
			if(currentBehaviour == MapPaintBehaviour.Delete_Chunk){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.DELETE_CHUNK),TidyTooltips.MAP_CREATOR_DELETE_CHUNK),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Delete_Chunk);
			}
			
			GUI.color = c;
			
			if(currentBehaviour == MapPaintBehaviour.DrawPath){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.DRAW_PATH),TidyTooltips.MAP_CREATOR_DRAW_PATH),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.DrawPath);
			}
			
			GUI.color = c;
			
			EditorGUILayout.EndHorizontal();
			
			EditorGUILayout.BeginHorizontal();
									
			/*if(currentBehaviour == MapPaintBehaviour.Edit_Functions){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.EDIT_FUNCTIONS),TidyTooltips.MAP_CREATOR_EDIT_FUNCTIONS),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Edit_Functions);
			}
			
			GUI.color = c;*/
			
			if(currentBehaviour == MapPaintBehaviour.Paint_Background){
				GUI.color = Color.green;
			}
			
			if(GUILayout.Button(new GUIContent(GetIcon(TidyFileNames.PAINT_BACKGROUND),TidyTooltips.MAP_CREATOR_PAINT_BACKGROUND),GUILayout.Width(btnWidth),GUILayout.Height(btnWidth))){
				SetPaintBehaviour(MapPaintBehaviour.Paint_Background);
			}
			
			GUI.color = c;
			
			EditorGUILayout.EndHorizontal();
			
			
			
		}
		
		Dictionary<string,bool> mapCodeFolds = new Dictionary<string, bool>();
		Dictionary<string,int> mapLayerVisibility = new Dictionary<string, int>();
		Dictionary<string,bool> mapViewAll = new Dictionary<string, bool>();
		
		Dictionary<string,bool> mapUnlockAll = new Dictionary<string, bool>();
		Dictionary<string,bool> mapLayerLock = new Dictionary<string, bool>();
		
		bool IsLayerUnlocked(Block block){
			
			if(block == null){
				return false;
			}
			
			if(block.blockMap == null){
				return false;
			}
			
			if(!mapUnlockAll.ContainsKey(block.blockMap.name)){
				mapUnlockAll.Add(block.blockMap.name,true);
			}
			
			if(mapUnlockAll[block.blockMap.name]){
				return true;
			}
			
			if(mapLayerLock.ContainsKey(block.blockMap.name + "_" + block.depth)){
				return mapLayerLock[block.blockMap.name + "_" + block.depth];
			}
			
			return false;
		}
		
		bool IsLayerUnlocked(MapChunk chunk){
				
			if(!mapUnlockAll.ContainsKey(chunk.parentMap.name)){
				mapUnlockAll.Add(chunk.parentMap.name,true);
			}
			
			if(mapUnlockAll[chunk.parentMap.name]){
				return true;
			}
			
			if(mapLayerLock.ContainsKey(chunk.parentMap.name + "_" + chunk.depth)){
				return mapLayerLock[chunk.parentMap.name + "_" + chunk.depth];
			}
			
			return false;
		}
			
		public void HideAllBlocksOfType(string name){
			
			Block[] targetBlocks = GameObject.FindObjectsOfType(typeof(Block)) as Block[];
			
			for(int i = 0; i < targetBlocks.Length; i++){
				
				if(targetBlocks[i].name == name){
					
					targetBlocks[i].Hide();
					
				}
				
			}
			
		}
		
		public void RefreshExistentBlockMaps(){
			
			existentBlockMaps = GameObject.FindObjectsOfType(typeof(BlockMap)) as BlockMap[];
		
			for(int i = 0; i < existentBlockMaps.Length; i++){
				
				if(!mapCodeFolds.ContainsKey(existentBlockMaps[i].name)){
					mapCodeFolds.Add(existentBlockMaps[i].name,false);
				}
				
				if(!mapViewAll.ContainsKey(existentBlockMaps[i].name)){
					mapViewAll.Add(existentBlockMaps[i].name,true);
				}
				
				if(!mapLayerVisibility.ContainsKey(existentBlockMaps[i].name)){
					mapLayerVisibility.Add(existentBlockMaps[i].name,0);
				}
				
				if(!mapUnlockAll.ContainsKey(existentBlockMaps[i].name)){
					mapUnlockAll.Add(existentBlockMaps[i].name,true);
				}
			}
			
		}
		
		
		void DrawVisibilityForMap(BlockMap map){
			
			Color c = GUI.color;
			
			bool viewAll = mapViewAll[map.name];
			
			if(viewAll){
				GUI.color = Color.green;
			}
			
			EditorGUILayout.BeginHorizontal();
			
			if(GUILayout.Button("View All Layers")){
				if(!viewAll){
					map.ShowAllLayers();
					mapViewAll[map.name] = true;
				}
			}
			
			GUI.color = c;
			
			if(!mapUnlockAll.ContainsKey(map.name)){
				mapUnlockAll.Add(map.name,true);
			}
			
			mapUnlockAll[map.name] = EditorGUILayout.Toggle(mapUnlockAll[map.name]);
			
			if(mapUnlockAll[map.name]){
				mapLayerLock.Clear();
			}
			
			EditorGUILayout.EndHorizontal();
			
			if(!mapLayerVisibility.ContainsKey(map.name)){
				mapLayerVisibility.Add(map.name,0);
			}
			
			int viewedLayer = mapLayerVisibility[map.name];
			
			for(int i = map.mapLowerDepth; i <= map.mapUpperDepth; i++){
				
				if(!viewAll && viewedLayer == i){
					GUI.color = Color.green;
				}
			
				EditorGUILayout.BeginHorizontal();
				
				if(GUILayout.Button("Layer " + i)){
					if(viewedLayer != i || viewAll){
						map.ShowLayer(i);
						mapLayerVisibility[map.name] = i;
						mapViewAll[map.name] = false;
					}
				}
				
				GUI.color = c;
				
				if(!mapLayerLock.ContainsKey(map.name + "_" + i)){
					mapLayerLock.Add(map.name + "_" + i,false);
				}
				
				mapLayerLock[map.name + "_" + i] = EditorGUILayout.Toggle(mapLayerLock[map.name + "_" + i]);
				
				if(mapLayerLock[map.name + "_" + i]){
					
					
					mapUnlockAll[map.name] = false;
				}
				
				EditorGUILayout.EndHorizontal();
				
			}
			
		}
		
		void DrawMapVisibilityPanel(){
			
			EditorGUILayout.BeginVertical();
			
			GUILayout.Label(TidyMessages.MAP_CREATOR_VISIBILITY_HEADER,EditorStyles.boldLabel);
			
			GUILayout.Space(10.0f);
			
			EditorGUILayout.BeginHorizontal();
			
			GUILayout.FlexibleSpace();
			
			if(GUILayout.Button("Refresh")){
				RefreshExistentBlockMaps();
			}
			
			GUILayout.Space(20.0f);
			
			EditorGUILayout.EndHorizontal();
			
			if(existentBlockMaps == null || existentBlockMaps.Length == 0){
				GUILayout.Label(TidyMessages.MAP_CREATOR_NO_MAPS_EXIST);
				EditorGUILayout.EndVertical();
				return;
			}
			
			bool needRefresh = false;
			
			for(int i = 0; i < existentBlockMaps.Length; i++){
				
				if(existentBlockMaps[i] != null){
						
					if(!mapCodeFolds.ContainsKey(existentBlockMaps[i].name)){
						mapCodeFolds.Add(existentBlockMaps[i].name,true);
					}
					
					bool fold = EditorGUILayout.Foldout(mapCodeFolds[existentBlockMaps[i].name],TidyMessages.MAP_CREATOR_LAYER_VISIBILITY_PREFIX + existentBlockMaps[i].name + TidyMessages.MAP_CREATOR_LAYER_VISIBILITY_SUFFIX);
					
					mapCodeFolds[existentBlockMaps[i].name] = fold;
					
					if(fold){
						
						DrawVisibilityForMap(existentBlockMaps[i]);
						
					}
				
				}
				else{
					needRefresh = true;
				}
			
				GUILayout.Space(10.0f);
				
			}
			
			
			EditorGUILayout.EndVertical();
						
			if(needRefresh){
								
				RefreshExistentBlockMaps();
			}
			
			/*string label = "Map bounds: No map selected";
			
			if(focusMap != null){
				label = "Map bounds: " + focusMap.mapLowerDepth + " to " + focusMap.mapUpperDepth;
			
				
				GUILayout.Label(label);
				
				GUILayout.Label("Display level:");
				
				bool cacheShow = EditorGUILayout.Toggle("Show all layers? ",showAllLayers);
				
				if(cacheShow != showAllLayers){
					showAllLayers = cacheShow;
					
					if(showAllLayers){
						focusMap.ShowAllLayers();	
					}
				}
				
				
				if(!showAllLayers){
					int cacheLayer = EditorGUILayout.IntSlider(displayLayer,focusMap.mapLowerDepth,focusMap.mapUpperDepth);
					
					if(cacheLayer != displayLayer){
						
						displayLayer = cacheLayer;
						
						focusMap.ShowLayer(displayLayer);
						
					}
				}
			}*/
			
		}
		
		float CalculateNFromRoundness(float raw_n, float roundness){
			
			float r_n = -Mathf.Pow(raw_n - (Mathf.Sqrt(1)),2.0f)+1.0f;
			
			//-((x-(sqrt(1)))^2)+1
				
			float n = Mathf.Lerp(raw_n,r_n,roundness);
			
			return n;
			
		}
		
		public void SetEntireMapDirty(BlockMap map){
			
			if(map.functionalOverlay != null){
				EditorUtility.SetDirty(map.functionalOverlay);
			}
			
			for(int i = 0; i < map.map.Length; i++){
				
				ChunkSet cs = map.map[i];
				
				if(cs != null){
					
					if(cs.HasChunks()){
						
						for(int j = 0; j < cs.chunkSet.Length; j++){
							
							MapChunk m = cs.chunkSet[j].chunk;
							
							if(m != null){
							
								EditorUtility.SetDirty(m);
								
								if(m.chunkPieces != null){
								
									for(int k = 0; k < m.chunkPieces.Length; k++){
										
										Block b = m.chunkPieces[k];
										
										if(b != null){
											EditorUtility.SetDirty(b);
										}
										
									}
									
								}
								
							}
							
						}
						
					}
					
				}
				
			}
			
		}
		
		void HandleBlockMove(Block moveBlock, Vector3 previousPosition){
			
			if(blockSelectionRadius == 0){
				return;
			}
			
			Vector3 movementDifference = moveBlock.transform.localPosition - previousPosition;
			
			if(movementDifference.z == 0.0f){
				return;
			}
					
			Vector2 bCoords = new Vector2(moveBlock.x,moveBlock.y);
			
			int depth = moveBlock.depth;
			
			for(int x = moveBlock.x - (blockSelectionRadius); x <= moveBlock.x + (blockSelectionRadius); x++){
				for(int y = moveBlock.y - (blockSelectionRadius); y <= moveBlock.y + (blockSelectionRadius); y++){
					
					if(x == moveBlock.x && y == moveBlock.y){
						continue;
					}
					
					Block b = focusMap.GetBlockAt(x,y,depth);
					
					if(b != null){
						
						float n = 0.0f;
						
						if(block_move_selection == TidyBlockMapCreator.Block_Move_Selection.Circle){
							
							float dist = Mathf.Abs(Vector2.Distance(new Vector2(b.x,b.y),bCoords));
							
							n = 1.0f - ((dist) / (float)(blockSelectionRadius+1.0f));
							
							if(n < 0.0f){
								continue;
							}
							
						}
						else{
						
							int diff = 0;
							
							int x_diff = Mathf.Abs(moveBlock.x - b.x);
							int y_diff = Mathf.Abs(moveBlock.y - b.y);
							
							if(x_diff < y_diff){
								diff = y_diff;
							}
							else{
								diff = x_diff;
							}
							
							n = 1.0f - ((float)(diff) / ((float)blockSelectionRadius+1.0f));
						}
						
						n = CalculateNFromRoundness(n,blockMoveRoundness);
						
						Vector3 pos = b.transform.localPosition;
						
						pos.z += movementDifference.z * n;
						
						b.transform.localPosition = pos;
											
					}
					
				}
			}
			
		}
		
	}
}



