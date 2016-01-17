using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using DopplerInteractive.TidyTileMapper.IconManagement;
using DopplerInteractive.TidyTileMapper.Utility;
using DopplerInteractive.TidyTileMapper.Words;

namespace DopplerInteractive.TidyTileMapper.Editors{
	
	public class TidyBlockEditor : IEditorWindow
	{
		//Block Editing logic
		//The invisibly-instantiated working block
		public OrientedBlock workingBlock = null;
		//The prefab of that block
		public OrientedBlock workingBlockPrefab = null;
		
		//Has this block changed?
		bool hasWorkingBlockChanged = false;
		
		string blockName = "";
						
		//Editor Logic
		EditorWindow parentWindow;
				
		Texture2D itemSelectedTexture = null;
		
		bool initialPreviewGenerated = true;
						
		//Icon and preview management
		PreviewDictionary previews;
		TextureDictionary icons;
		
		//GUI Logic
		
		Vector2 blockWindowScrollPos = Vector2.zero;
		Vector2 windowScrollPos = Vector2.zero;
				
		TidyBlockMapCreator mapCreator;
		
		List<OrientedBlock> workingBlocks = null;
						
		bool fold_basicBlocks = true;
		bool fold_flatSurfaces = true;
		bool fold_blockLines = true;
		bool fold_protudingBlocks = true;				
		bool fold_corners = true;
		bool fold_innerCorners = true;
		bool fold_slopedEdges = true;
		bool fold_intersections = true;
		
		//New Orientations as at 2nd January 2013
		//100% of the thanks go to Alex Evangelou from Angry Fish Studios
		//http://www.angryfishstudios.com/
		//...who provided all of the icons, the names, and kindly requested these orientations be added
		//like... 8 months ago; and never sent me an angry email when I (repeatedly) forgot to do it.
		bool fold_borders = true;
		bool fold_gates = true;
		bool fold_forks = true;
		bool fold_funnels = true;
		bool fold_tapers = true;
		
		bool fold_blockOptions = false;
						
		public void Initialize(EditorWindow parentWindow){}
		
		public void Initialize(EditorWindow parentWindow, TidyBlockMapCreator mapCreator){
			
			this.parentWindow = parentWindow;
						
			this.mapCreator = mapCreator;
			
			itemSelectedTexture = GetIcon(TidyFileNames.ICON_ITEM_SELECTED);
			
			PopulateBlockList();
			
			this.parentWindow.name = TidyMessages.BLOCK_EDITOR_WINDOW_NAME;
						
		}
		
		void PopulateBlockList(){
			
			Block[] b = TidyEditorUtility.GetCurrentBlocks();
						
			workingBlocks = new List<OrientedBlock>();
			
			if(b != null){
				
				for(int i = 0; i < b.Length; i++){
					
					workingBlocks.Add(b[i] as OrientedBlock);
					
				}
				
			}
			
			workingBlocks.Insert(0,TidyEditorUtility.GetNullBlock() as OrientedBlock);
			
			//Working block
			if(workingBlocks.Count > 0){
				
				HandleBlockChange(workingBlocks[0],false);
		
			}
		}
			
		public void Destroy(){
		
			if(HasWorkingBlockChanged()){
				if(EditorUtility.DisplayDialog(TidyMessages.BLOCK_EDITOR_CLOSE_DIALOG,
				                               TidyMessages.BLOCK_EDITOR_CLOSE_PROMPT,
				                               TidyMessages.BLOCK_EDITOR_CLOSE_CONFIRM,
				                               TidyMessages.BLOCK_EDITOR_CLOSE_REJECT)){
					
					HandleBlockChange(null,true);
					
				}
				else{
					
					HandleBlockChange(null,false);
					
				}
			}
			else{
				
				HandleBlockChange(null,false);
				
			}
			
			AssetDatabase.Refresh();
			
		}
		
		public void Update(){
		
			
			if(EditorApplication.isPlayingOrWillChangePlaymode){
				
				parentWindow.ShowNotification(new GUIContent(TidyMessages.BLOCK_EDITOR_NO_PLAY));
				
			}
			
			EditorApplication.isPlaying = false;
			
			
			//We need to update this until the sync is finished
			if(UpdatePreviews()){
				
				parentWindow.ShowNotification(new GUIContent(TidyMessages.PREVIEW_GENERATING_PREVIEWS_TITLE));
				
				parentWindow.Repaint();
			}
			else{
				
				parentWindow.RemoveNotification();
				
			}
			
		}
		
		public void DrawScene(){
			
		}
		
		void CreateNewBlock(){
				
			GameObject newBlock = new GameObject("Block_"+(workingBlocks.Count+1));
			
			newBlock.gameObject.hideFlags = HideFlags.HideInHierarchy;
			
			//We need to set the block up now
			newBlock.AddComponent<OrientedBlock>();
			newBlock.AddComponent<BoxCollider>();
			
			//Create a prefab
			//UnityEngine.Object prefab = EditorUtility.CreateEmptyPrefab("Assets/"+TidyEditorUtility.blockPath + "/" + newBlock.name + ".prefab");
			
			UnityEngine.Object prefab = PrefabUtility.CreateEmptyPrefab("Assets/"+TidyEditorUtility.blockPath + "/" + newBlock.name + ".prefab");
			
			//GameObject newPrefab = EditorUtility.ReplacePrefab(newBlock,prefab);
			
			GameObject newPrefab = PrefabUtility.ReplacePrefab(newBlock,prefab);
			
			OrientedBlock po = newPrefab.GetComponent<OrientedBlock>();
						
			workingBlocks.Add(po);
			
			if(HasWorkingBlockChanged()){
				
				GetPreviewForGameObject(workingBlockPrefab.GetDefaultBlock(),true);
				
				if(EditorUtility.DisplayDialog(TidyMessages.BLOCK_EDITOR_SWITCH_DIALOG,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_PROMPT,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_CONFIRM,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_REJECT)){
					HandleBlockChange(po,true);
				}
				else{
					HandleBlockChange(po,false);
				}
			}
			else{
				
				HandleBlockChange(po,false);
			}
			
			initialPreviewGenerated = false;
			
			Editor.DestroyImmediate(newBlock);
			
			mapCreator.RefreshWorkingBlockMenu();
			
		}
		
		void SaveBlock(OrientedBlock block, GameObject blockPrefab){
			
			//rename the screenshot
					
			if(blockName != block.name){
			
				string path = TidyEditorUtility.previewPath+"/"+block.name+".png";
						
				AssetDatabase.RenameAsset("Assets/"+TidyEditorUtility.blockPath+"/"+block.name+".prefab",blockName);
				
				//rename the block
				workingBlock.name = blockName;
				
				AssetDatabase.RenameAsset("Assets/"+path,block.name);
									
			}
			//save the created object over the existing prefab
			//EditorUtility.ReplacePrefab(block.gameObject,blockPrefab.gameObject);
			
			mapCreator.HideAllBlocksOfType(blockPrefab.name);
			
			PrefabUtility.DisconnectPrefabInstance(block.gameObject);
			
			PrefabUtility.ReplacePrefab(block.gameObject,blockPrefab.gameObject);
					
			PrefabUtility.RevertPrefabInstance(block.gameObject);
			
			mapCreator.RefreshWorkingBlockMenu();
			
		}
		
		//Handle a change in block selection
		void HandleBlockChange(OrientedBlock newWorkingBlock, bool saveChanges){
			
			newObjects.Clear();
						
			if(workingBlock != null){
				
				workingBlock.gameObject.hideFlags = 0;
				
				if(saveChanges){
					
					SaveBlock(workingBlock,workingBlockPrefab.gameObject);
					
				}
								
				Editor.DestroyImmediate(workingBlock.gameObject);
			
			}
			
			if(newWorkingBlock != null){
			
				//Set the new block
				workingBlockPrefab = newWorkingBlock;
			
				//We'll actually replace this with hide flags
				//Once we're sure it works
				
				//workingBlock = EditorUtility.InstantiatePrefab(workingBlockPrefab) as OrientedBlock;
				
				workingBlock = PrefabUtility.InstantiatePrefab(workingBlockPrefab) as OrientedBlock;
				
				//Hide in the inspector
				//workingBlock.gameObject.hideFlags = HideFlags.HideInHierarchy;
				
				//Disappear!
				
#if UNITY_4_0
				workingBlock.gameObject.SetActive(false);
#else
				workingBlock.gameObject.SetActiveRecursively(false);
#endif
				
				blockName = workingBlock.name;
					
			}
			
			hasWorkingBlockChanged = false;
			
		}
		
		bool HasWorkingBlockChanged(){
			
			return (hasWorkingBlockChanged || ((workingBlock != null) && (blockName != workingBlock.name)));
			
		}
		
		//Rect bRect = new Rect(0.0f,0.0f,0.0f,0.0f);
		
		public void DrawWindow(){
					
			EditorGUILayout.BeginHorizontal();
						
			EditorGUILayout.BeginVertical();
			
			GUILayout.Label(TidyMessages.BLOCK_EDITOR_BLOCKLIST_TITLE);
			
			if(GUILayout.Button(
			                    new GUIContent(TidyMessages.BLOCK_EDITOR_ADD_BLOCK,
			                                   TidyMessages.BLOCK_EDITOR_ADD_BLOCK_TOOLTIP),GUILayout.Width(125))){
				CreateNewBlock();
			}
			
			//GUI.Box(bRect,"");
			
			windowScrollPos = GUILayout.BeginScrollView(windowScrollPos,GUILayout.Width(125));
			
			Rect br = EditorGUILayout.BeginVertical();
			
			GUI.Box(br,"");
			
			bool refreshBlocks = false;
			
			for(int i = 0; i < workingBlocks.Count; i++){
				
				if(workingBlocks[i] == null){
					
					refreshBlocks = true;
						
					continue;
					
				}
				
				GUILayout.Label(workingBlocks[i].name);
				
				if(GUILayout.Button("",GUILayout.Width(100),GUILayout.Height(100))){
					
					//Here we go
					if(workingBlocks[i] != workingBlockPrefab){
						
						if(HasWorkingBlockChanged()){
							
							GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
							
							if(EditorUtility.DisplayDialog(TidyMessages.BLOCK_EDITOR_SWITCH_DIALOG,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_PROMPT,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_CONFIRM,
				                               TidyMessages.BLOCK_EDITOR_SWITCH_REJECT)){
								HandleBlockChange(workingBlocks[i],true);
							}
							else{
								HandleBlockChange(workingBlocks[i],false);
							}
						}
						else{
							
							HandleBlockChange(workingBlocks[i],false);
						}
				
					}
						
												
				}
				
				Rect r = GUILayoutUtility.GetLastRect();
												
				Texture2D preview = GetPreviewForGameObject(workingBlocks[i].GetDefaultBlock(),false);
												
				if(preview != null){
					GUI.DrawTexture(r,preview,ScaleMode.ScaleAndCrop,false);
					
					if(workingBlocks[i] == workingBlockPrefab){
						GUI.DrawTexture(r,itemSelectedTexture,ScaleMode.StretchToFill,true);
					}
				}
				
				
			}
			
			GUILayout.FlexibleSpace();
			
			EditorGUILayout.EndVertical();
			
			GUILayout.EndScrollView();
			
			if(refreshBlocks){
				PopulateBlockList();
			}
			
			if(Event.current.type == EventType.Repaint){
				//bRect = GUILayoutUtility.GetLastRect();
			}
														
			EditorGUILayout.EndVertical();
			
			if(workingBlock != null){
			
				EditorGUILayout.BeginVertical();
				
				EditorGUILayout.BeginHorizontal();
				
				GUILayout.Label(TidyMessages.BLOCK_EDITOR_BLOCK_NAME);
				
				bool emptyBlock = TidyEditorUtility.IsEmptyBlock(workingBlock);
				
				if(emptyBlock){
					GUI.enabled = false;
				}
				
				blockName = GUILayout.TextField(blockName,50,GUILayout.Width(100));
				
				blockName = blockName.Replace(".","");
				blockName = blockName.Replace("/","");
				blockName = blockName.Replace("\\","");
				
				
				GUILayout.FlexibleSpace();
				
				if(!hasWorkingBlockChanged && !emptyBlock){
				
					GUI.enabled = false;
					
				}
				
				if(GUILayout.Button(new GUIContent(TidyMessages.BLOCK_EDITOR_SAVE_BUTTON,
				                                   TidyMessages.BLOCK_EDITOR_SAVE_BUTTON_TOOLTIP))){
					
					SaveBlock(workingBlock,workingBlockPrefab.gameObject);
					
					GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
					
					hasWorkingBlockChanged = false;
						
				}
				
				if(!hasWorkingBlockChanged && !emptyBlock){
				
					GUI.enabled = true;
					
				}
				
				EditorGUILayout.EndHorizontal();
				
				EditorGUILayout.Space();
				
				
				fold_blockOptions = EditorGUILayout.Foldout(fold_blockOptions, TidyMessages.BLOCK_EDITOR_BLOCK_OPTIONS);
				
				if(fold_blockOptions){
					
					EditorGUILayout.BeginVertical();
					
					bool cacheOption = workingBlock.actAsEmptyBlock;
									
					EditorGUI.indentLevel = 2;
					
					workingBlock.actAsEmptyBlock = EditorGUILayout.Toggle(new GUIContent(
					                                                                     TidyMessages.BLOCK_EDITOR_ACT_AS_EMPTY_BLOCK,
					                                                                     TidyMessages.BLOCK_EDITOR_ACT_AS_EMPTY_BLOCK_TOOLTIP),workingBlock.actAsEmptyBlock);
					
					if(workingBlock.actAsEmptyBlock != cacheOption){
						hasWorkingBlockChanged = true;
					}
					
					bool cacheColliderOption = workingBlock.retainCollider;
														
					workingBlock.retainCollider = EditorGUILayout.Toggle(new GUIContent(TidyMessages.BLOCK_EDITOR_RETAIN_COLLIDER,
					                                                                    TidyMessages.BLOCK_EDITOR_RETAIN_COLLIDER_TOOLTIP),workingBlock.retainCollider);
					
					if(workingBlock.retainCollider != cacheColliderOption){
						hasWorkingBlockChanged = true;
					}
					
					float x_offset = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_X_OFFSET,
					                                                                    TidyMessages.BLOCK_EDITOR_X_OFFSET_TOOLTIP),
					                                                   workingBlock.x_offset);
					
				
					float y_offset = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_Y_OFFSET,
					                                                                    TidyMessages.BLOCK_EDITOR_Y_OFFSET_TOOLTIP),
					                                                   workingBlock.y_offset);
					
					float z_offset = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_Z_OFFSET,
					                                                                    TidyMessages.BLOCK_EDITOR_Z_OFFSET_TOOLTIP),
					                                                   workingBlock.z_offset);
					
										
					float x_rotation = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_X_ROTATION,
					                                                                    TidyMessages.BLOCK_EDITOR_X_ROTATION_TOOLTIP),
					                                                   workingBlock.x_rotation);
					
				
					float y_rotation = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_Y_ROTATION,
					                                                                    TidyMessages.BLOCK_EDITOR_Y_ROTATION_TOOLTIP),
					                                                   workingBlock.y_rotation);
					
					float z_rotation = EditorGUILayout.FloatField(new GUIContent(TidyMessages.BLOCK_EDITOR_Z_ROTATION,
					                                                                    TidyMessages.BLOCK_EDITOR_Z_ROTATION_TOOLTIP),
					                                                   workingBlock.z_rotation);
					
					if(x_offset != workingBlock.x_offset || y_offset != workingBlock.y_offset || z_offset != workingBlock.z_offset
					   || x_rotation != workingBlock.x_rotation || y_rotation != workingBlock.y_rotation || z_rotation != workingBlock.z_rotation){
						workingBlock.SetOffset(new Vector3(x_offset,y_offset,z_offset),
						                       new Vector3(x_rotation,y_rotation,z_rotation));
						hasWorkingBlockChanged = true;
						
					}
					
					EditorGUI.indentLevel = 0;
					
					EditorGUILayout.EndVertical();
						
				}
				
				EditorGUILayout.Space();
				
				blockWindowScrollPos = GUILayout.BeginScrollView(blockWindowScrollPos);
				
				EditorGUILayout.BeginVertical(GUILayout.ExpandWidth(true));
				
				fold_basicBlocks = EditorGUILayout.Foldout(fold_basicBlocks, TidyMessages.BLOCK_EDITOR_BASIC_BLOCKS);
				
				EditorGUILayout.Space();
				
				if(fold_basicBlocks){
					DrawBlockSet(workingBlock.Centre_Alone, BlockOrientation.Centre_Alone);
					DrawBlockSet(workingBlock.Centre_Surrounded, BlockOrientation.Centre_Surrounded);
				}
					
				fold_flatSurfaces = EditorGUILayout.Foldout(fold_flatSurfaces, TidyMessages.BLOCK_EDITOR_FLAT_SURFACES);
				
				EditorGUILayout.Space();
				
				if(fold_flatSurfaces){
					DrawBlockSet(workingBlock.Top_Surrounded, BlockOrientation.Top_Surrounded);
					DrawBlockSet(workingBlock.Bottom_Surrounded, BlockOrientation.Bottom_Surrounded);								
					DrawBlockSet(workingBlock.Left_Surrounded, BlockOrientation.Left_Surrounded);								
					DrawBlockSet(workingBlock.Right_Surrounded, BlockOrientation.Right_Surrounded);
				}
					
				fold_blockLines = EditorGUILayout.Foldout(fold_blockLines, TidyMessages.BLOCK_EDITOR_BLOCK_LINES);
				
				EditorGUILayout.Space();
				
				if(fold_blockLines){
					DrawBlockSet(workingBlock.Centre_Horizontal_Surrounded, BlockOrientation.Centre_Horizontal_Surrounded);
					DrawBlockSet(workingBlock.Centre_Vertical_Surrounded, BlockOrientation.Centre_Vertical_Surrounded);
					DrawBlockSet(workingBlock.Right_Diagonal, BlockOrientation.Right_Diagonal);
					DrawBlockSet(workingBlock.Left_Diagonal, BlockOrientation.Left_Diagonal);
				}
					
				fold_protudingBlocks = EditorGUILayout.Foldout(fold_protudingBlocks, TidyMessages.BLOCK_EDITOR_PROTRUDING_BLOCKS);
				
				EditorGUILayout.Space();
				
				if(fold_protudingBlocks){
					DrawBlockSet(workingBlock.Top_Alone, BlockOrientation.Top_Alone);
					DrawBlockSet(workingBlock.Bottom_Alone, BlockOrientation.Bottom_Alone);
					DrawBlockSet(workingBlock.Left_Alone, BlockOrientation.Left_Alone);
					DrawBlockSet(workingBlock.Right_Alone, BlockOrientation.Right_Alone);
				}
				
				fold_corners = EditorGUILayout.Foldout(fold_corners, TidyMessages.BLOCK_EDITOR_CORNERS);
				
				EditorGUILayout.Space();
				
				if(fold_corners){
					DrawBlockSet(workingBlock.Top_Left, BlockOrientation.Top_Left);
					DrawBlockSet(workingBlock.Top_Right, BlockOrientation.Top_Right);
					DrawBlockSet(workingBlock.Bottom_Left, BlockOrientation.Bottom_Left);
					DrawBlockSet(workingBlock.Bottom_Right, BlockOrientation.Bottom_Right);
				}
					
				fold_innerCorners = EditorGUILayout.Foldout(fold_innerCorners, TidyMessages.BLOCK_EDITOR_INNER_CORNERS);
				
				EditorGUILayout.Space();
				
				if(fold_innerCorners){
					DrawBlockSet(workingBlock.Top_Right_Inner_Corner, BlockOrientation.Top_Right_Inner_Corner);
					DrawBlockSet(workingBlock.Bottom_Right_Inner_Corner, BlockOrientation.Bottom_Right_Inner_Corner);
					DrawBlockSet(workingBlock.Top_Left_Inner_Corner, BlockOrientation.Top_Left_Inner_Corner);
					DrawBlockSet(workingBlock.Bottom_Left_Inner_Corner, BlockOrientation.Bottom_Left_Inner_Corner);
				}
				
				fold_slopedEdges = EditorGUILayout.Foldout(fold_slopedEdges, TidyMessages.BLOCK_EDITOR_SLOPED_EDGES);
				
				EditorGUILayout.Space();
				
				if(fold_slopedEdges){
					DrawBlockSet(workingBlock.Lower_Left_Filled_Diagonal, BlockOrientation.Lower_Left_Filled_Diagonal);
					DrawBlockSet(workingBlock.Lower_Right_Filled_Diagonal, BlockOrientation.Lower_Right_Filled_Diagonal);
					DrawBlockSet(workingBlock.Upper_Right_Filled_Diagonal, BlockOrientation.Upper_Right_Filled_Diagonal);
					DrawBlockSet(workingBlock.Upper_Left_Filled_Diagonal, BlockOrientation.Upper_Left_Filled_Diagonal);
				}
				
				fold_intersections = EditorGUILayout.Foldout(fold_intersections, TidyMessages.BLOCK_EDITOR_INTERSECTIONS);
				
				EditorGUILayout.Space();
				
				if(fold_intersections){
					DrawBlockSet(workingBlock.Cross_Hub, BlockOrientation.Cross_Hub);
					DrawBlockSet(workingBlock.Upward_T_Hub, BlockOrientation.Upward_T_Hub);
					DrawBlockSet(workingBlock.Downward_T_Hub, BlockOrientation.Downward_T_Hub);
					DrawBlockSet(workingBlock.Leftward_T_Hub, BlockOrientation.Leftward_T_Hub);
					DrawBlockSet(workingBlock.Rightward_T_Hub, BlockOrientation.Rightward_T_Hub);
				}
					
				EditorGUILayout.Space();
				
				if(fold_borders){
					DrawBlockSet(workingBlock.Border_Bottom_Right, BlockOrientation.Border_Bottom_Right);
					DrawBlockSet(workingBlock.Border_Bottom_Left, BlockOrientation.Border_Bottom_Left);
					DrawBlockSet(workingBlock.Border_Top_Left, BlockOrientation.Border_Top_Left);
					DrawBlockSet(workingBlock.Border_Top_Right, BlockOrientation.Border_Top_Right);
				}
					
				EditorGUILayout.Space();
				
				if(fold_gates){
					DrawBlockSet(workingBlock.Diagonal_Gate_Left, BlockOrientation.Diagonal_Gate_Left);
					DrawBlockSet(workingBlock.Diagonal_Gate_Right, BlockOrientation.Diagonal_Gate_Right);
				}
					
				EditorGUILayout.Space();
				
				if(fold_forks){
					DrawBlockSet(workingBlock.Fork_Top_Left, BlockOrientation.Fork_Top_Left);
					DrawBlockSet(workingBlock.Fork_Top_Right, BlockOrientation.Fork_Top_Right);
					DrawBlockSet(workingBlock.Fork_Bottom_Right, BlockOrientation.Fork_Bottom_Right);
					DrawBlockSet(workingBlock.Fork_Bottom_Left, BlockOrientation.Fork_Bottom_Left);
				}
					
				EditorGUILayout.Space();
				
				if(fold_funnels){
					DrawBlockSet(workingBlock.Funnel_Left, BlockOrientation.Funnel_Left);
					DrawBlockSet(workingBlock.Funnel_Right, BlockOrientation.Funnel_Right);
					DrawBlockSet(workingBlock.Funnel_Top, BlockOrientation.Funnel_Top);
					DrawBlockSet(workingBlock.Funnel_Bottom, BlockOrientation.Funnel_Bottom);
				}
					
				EditorGUILayout.Space();
				
				if(fold_tapers){
					DrawBlockSet(workingBlock.Taper_Bottom_Left, BlockOrientation.Taper_Bottom_Left);
					DrawBlockSet(workingBlock.Taper_Bottom_Right, BlockOrientation.Taper_Bottom_Right);
					DrawBlockSet(workingBlock.Taper_Left_Bottom, BlockOrientation.Taper_Left_Bottom);
					DrawBlockSet(workingBlock.Taper_Left_Top, BlockOrientation.Taper_Left_Top);
					DrawBlockSet(workingBlock.Taper_Right_Bottom, BlockOrientation.Taper_Right_Bottom);
					DrawBlockSet(workingBlock.Taper_Right_Top, BlockOrientation.Taper_Right_Top);
					DrawBlockSet(workingBlock.Taper_Top_Left, BlockOrientation.Taper_Top_Left);
					DrawBlockSet(workingBlock.Taper_Top_Right, BlockOrientation.Taper_Top_Right);
				}
					
				EditorGUILayout.Space();
				
				
				EditorGUILayout.EndVertical();
				
				GUILayout.EndScrollView();
				
				EditorGUILayout.EndVertical();
				
				EditorGUILayout.EndHorizontal();
				
				if(emptyBlock){
					GUI.enabled = true;
				}
			}
			
		}
		
		//A dictionary of all scroll positions
		//For each set
		//I love dictionaries, you know
		Dictionary<string,Vector2> scrollPositions = new Dictionary<string,Vector2>();
		//A dictionary of the object selections
		//For each set
		Dictionary<string,GameObject> newObjects = new Dictionary<string,GameObject>();
		
		//Dictionary<string,Rect> outlineRects = new Dictionary<string, Rect>();
		
		void DrawBlockSet(BlockSet blocks, BlockOrientation orientation){
			
			string orientationName = orientation.ToString();
			
			if(!scrollPositions.ContainsKey(orientationName)){
				scrollPositions.Add(orientationName,Vector2.zero);
			}
			
			
			EditorGUILayout.BeginHorizontal(GUILayout.ExpandWidth(false));
			
			EditorGUILayout.BeginVertical(GUILayout.Width(150));
			
			EditorGUILayout.BeginHorizontal(GUILayout.ExpandWidth(false));
		
			Texture2D icon = GetIcon(orientation.ToString());
			
			GUILayout.Label(new GUIContent("",orientationName),GUILayout.ExpandWidth(false), GUILayout.MinWidth(100),GUILayout.MinHeight(100));
						
			GUI.DrawTexture(GUILayoutUtility.GetLastRect(),icon,ScaleMode.ScaleAndCrop,false);
						
			bool isDefault = GUILayout.Toggle(blocks.IsDefault(),new GUIContent("",TidyMessages.BLOCK_EDITOR_BLOCKSET_IS_DEFAULT_TOOLTIP),GUILayout.Height(25));
			
			if(isDefault && !blocks.IsDefault()){
				
				workingBlock.ClearDefault();
				
				blocks.SetIsDefault(true);
								
				SetBlockDirty(workingBlock);
				
				hasWorkingBlockChanged = true;
				
				if(!initialPreviewGenerated){
					
					//there will be no other opportunity to refresh
					GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
					initialPreviewGenerated = true;
				}
				
			}
			
			if(!isDefault && blocks.IsDefault()){
				
				workingBlock.ClearDefault();
					
				hasWorkingBlockChanged = true;
				
				SetBlockDirty(workingBlock);
				
				if(!initialPreviewGenerated){
					
					//there will be no other opportunity to refresh
					GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
					initialPreviewGenerated = true;
				}
				
			}
			
			EditorGUILayout.EndHorizontal();
					
			if(!newObjects.ContainsKey(orientationName)){
				newObjects.Add(orientationName,null);
			}
			
			newObjects[orientationName] = EditorGUILayout.ObjectField(newObjects[orientationName],typeof(GameObject),false,GUILayout.Width(150)) as GameObject;
			
			//Change as at 30/12/2011: No more button, automatic addition			
			if(GUILayout.Button(new GUIContent(TidyMessages.BLOCK_EDITOR_BLOCKSET_ADD_VARIANT,
			                                   TidyMessages.BLOCK_EDITOR_BLOCKSET_ADD_VARIANT_TOOLTIP)) && (newObjects[orientationName] != null && !newObjects[orientationName].Equals(workingBlock.gameObject))){
											
				//GameObject newBlock = EditorUtility.InstantiatePrefab(newObjects[orientationName]) as GameObject;
				
				//PREFAB
				//GameObject newBlock = PrefabUtility.InstantiatePrefab(newObjects[orientationName]) as GameObject;
				//newBlock.name = newObjects[orientationName].name;
				//newBlock.SetActiveRecursively(false);
				
				workingBlock.Editor_AddToBlockSet(orientation,newObjects[orientationName]);
				
				if(workingBlock.GetDefaultSet() == null){
					
					blocks.SetIsDefault(true);
					
				}
				
				hasWorkingBlockChanged = true;
				
				if(!initialPreviewGenerated){
					
					//there will be no other opportunity to refresh
					GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
					initialPreviewGenerated = true;
				}
				
				SetBlockDirty(workingBlock);
				
				newObjects[orientationName] = null;
				
				mapCreator.RefreshAllMaps();
				
			}
			
			EditorGUILayout.EndVertical();
					
			Vector2 scrollPos = scrollPositions[orientationName];
			
			//scrollPos = GUILayout.BeginScrollView(scrollPos,GUILayout.MinHeight(130),GUILayout.ExpandWidth(true));
			
			Rect r = EditorGUILayout.BeginHorizontal(GUILayout.ExpandWidth(true),GUILayout.MinHeight(130));
					
			GUI.Box(r,"");
			
			if(blocks != null && blocks.blockSet != null){
						
				for(int i = 0; i < blocks.blockSet.Length; i++){
					
					DrawObjectPreview(blocks.blockSet[i],orientation);
				}
				
			}
			
			GUILayout.FlexibleSpace();
			
			EditorGUILayout.EndHorizontal();
			
			//GUILayout.EndScrollView();
			
			
			//GUI.Box(GUILayoutUtility.GetLastRect(),"");
			
			scrollPositions[orientationName] = scrollPos;
			
			EditorGUILayout.EndHorizontal();
		
			
		}
		
		void DrawObjectPreview(GameObject o, BlockOrientation orientation){
			
			if(o == null){
				
				return;
			}
			
			GUILayout.BeginVertical();
			
			Texture2D preview = GetPreviewForGameObject(o,false);
				
			GUILayout.BeginHorizontal(GUILayout.Width(100));
					
			string name = o.name;
			
			if(name.Length >= 12){
				
				name = name.Substring(0,9);
				name += "...";
				
			}
			
			GUILayout.Label(name);
			
			bool deleted = false;
			
			if(GUILayout.Button(new GUIContent(icons.GetTexture(TidyFileNames.ICON_DELETE_BLOCKPART),TidyMessages.BLOCK_EDITOR_BLOCKSET_REMOVE_VARIANT_TOOLTIP),GUILayout.Width(18),GUILayout.Height(18))){
				
				workingBlock.Editor_RemoveFromBlockSet(orientation,o);
				
				//PREFAB
				/*if(destroyObject != null){			
					Editor.DestroyImmediate(destroyObject,true);
				}*/
				
				SetBlockDirty(workingBlock);
				
				deleted = true;
				
				hasWorkingBlockChanged = true;
				
				if(!initialPreviewGenerated){
					
					//there will be no other opportunity to refresh
					GetPreviewForGameObject(workingBlock.GetDefaultBlock(),true);
					initialPreviewGenerated = true;
				}
			}
			
			GUILayout.EndHorizontal();
			
			if(!deleted){
				
				if(preview == null){
					GUILayout.Button(o.name,GUILayout.Width(100),GUILayout.Height(100));
				}
				else{
					GUILayout.Button(new GUIContent("",o.name),GUILayout.Width(100),GUILayout.Height(100));
					GUI.DrawTexture(GUILayoutUtility.GetLastRect(),preview,ScaleMode.ScaleAndCrop,false);
				}
			
			}
			GUILayout.EndVertical();
					
		}
				
		public EditorWindow GetParentWindow(){
			return parentWindow;
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
		
		public void OnSelectionChange ()
		{
		}
		
		public void SetBlockDirty(OrientedBlock workingBlock){
			
			EditorUtility.SetDirty(workingBlock);
						
		}
		
				
	}
	
	
	

}