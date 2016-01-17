using UnityEngine;
using UnityEditor;
using System;
using System.Collections.Generic;

namespace DopplerInteractive.TidyTileMapper.IconManagement{	
	
	public class PreviewDictionary : TextureDictionary{
		
		//We request previews from the editor in sequence
		//They are generated and have a slight pause between being returned
		List<GameObject> previewRequests = new List<GameObject>();
		Dictionary<string,GameObject> previewObjects = new Dictionary<string, GameObject>();
			
		bool[] allLightingSettings = null;
		
		List<string> pendingTextureImports = new List<string>();
		
		float lastUpdate = 0.0f;
		float updateRate = 2.0f;
		
		public PreviewDictionary(string sourceDirectory):base(sourceDirectory){}
		
		class PreviewRequest{
			
			public GameObject previewObject;
			public bool overrideExisting = false;
			
			public PreviewRequest(GameObject previewObject, bool overrideExisting){
				this.previewObject = previewObject;
				this.overrideExisting = overrideExisting;
			}
			
		}
		
		public Texture2D GetPreviewTexture(GameObject previewObject){
			
			if(previewObject == null){
				return null;
			}
			
			//Check to see if it already exists
			Texture2D texture = GetTexture(previewObject.name);
			
			if(texture != null){
			
				return texture;
			
			}
				
			//Check to see if it has been created and can be loaded
			
			if(!previewObjects.ContainsKey(previewObject.name)){
				
				LoadTextureForName(previewObject.name);
			
				texture = GetTexture(previewObject.name);	
			}
			
			if(texture != null){
			
				if(pendingTextureImports.Contains(previewObject.name)){
					pendingTextureImports.Remove(previewObject.name);
				}
				
				return texture;
			
			}
			
			RequestPreview(previewObject,false);
			
			return null;
			
		}
		
		public Texture2D GetPreviewTexture(GameObject previewObject, bool overwrite){
			
			if(previewObject == null){
				return null;
			}
			
			if(overwrite){
				
				if(previewObjects.ContainsKey(previewObject.name)){
				
					previewObjects.Remove(previewObject.name);
					pendingTextureImports.Remove(previewObject.name);
				}
				
				RequestPreview(previewObject,overwrite);
				
				return null;
			}
			
			return GetPreviewTexture(previewObject);
			
		}
		
		void RequestPreview(GameObject previewObject,bool overwrite){
				
			if((ContainsName(previewRequests,previewObject.name)||pendingTextureImports.Contains(previewObject.name)) && !overwrite){
				
				return;
			}
									
			previewRequests.Add(previewObject);
		
		}
				
		bool ContainsName(List<GameObject> collection, string name){
			
			for(int i = 0; i < collection.Count; i++){
				
				if(collection[i].name == name){
					return true;
				}
				
			}
			
			return false;
			
		}
		
		public bool UpdatePreviewImports(float deltaTime){
			
			deltaTime = 0.1f;
			
			if(IsUpdatingPreviewImport()){
				
				lastUpdate += deltaTime;
				
				if(lastUpdate >= updateRate){
				
					if(GeneratePreview(previewRequests[0])){
						previewRequests.RemoveAt(0);
						return true;
					}
					
					lastUpdate = 0.0f;
				}
			}
			else{
				
			}
			
			return false;
		}
		
		public bool IsUpdatingPreviewImport(){
			return (previewRequests.Count > 0);
		}
		
		void PlacePreviewCamera(GameObject cameraNode, GameObject previewObject){
			
			cameraNode.transform.localPosition = Vector3.zero;
			cameraNode.transform.forward = previewObject.transform.forward * -1;
			
			Vector3 m_dimensions = Vector3.zero;
			Vector3 r_dimensions = Vector3.zero;
			Vector3 c_dimensions = Vector3.zero;
			
			//Ideally, we want a mesh filter
			MeshFilter mf = previewObject.GetComponent<MeshFilter>();
			Renderer r = previewObject.GetComponent<Renderer>();
			Collider c = previewObject.GetComponent<Collider>();
			
			if(mf == null){
				mf = previewObject.GetComponentInChildren<MeshFilter>();
			}
			
			if(r == null){
				r = previewObject.GetComponentInChildren<Renderer>();
			}
			
			if(c == null){
				c = previewObject.GetComponentInChildren<Collider>();
			}
			
			if(mf != null){
				
				m_dimensions.y = mf.sharedMesh.bounds.size.y;
				m_dimensions.z = mf.sharedMesh.bounds.size.z;
				m_dimensions.x = mf.sharedMesh.bounds.size.x;
				
			}
			
			if(r != null){
				
				r_dimensions.y = r.bounds.size.y;
				r_dimensions.x = r.bounds.size.x;
				r_dimensions.z = r.bounds.size.z;
				
			}
			
			if(c != null){
				
				c_dimensions.y = c.bounds.size.y;
				c_dimensions.x = c.bounds.size.x;
				c_dimensions.z = c.bounds.size.z;
				
			}
			
			if(mf != null && (m_dimensions.x > 0.0f) && (m_dimensions.y > 0.0f)){
				
				Vector3 localPosition = cameraNode.transform.localPosition;
				
				float n = 0.0f;
				
				if(m_dimensions.z > m_dimensions.y){
					n = m_dimensions.z;
				}
				else{
					n = m_dimensions.y;
				}
				
				localPosition.z = n * 1.5f;
				
				localPosition.y = n * 1.5f;
												
				cameraNode.transform.localPosition = localPosition;
			}
			else
			if(r != null && (r_dimensions.x > 0.0f) && (r_dimensions.y > 0.0f)){
				
				Vector3 localPosition = cameraNode.transform.localPosition;
				
				float n = 0.0f;
				
				if(r_dimensions.z > r_dimensions.y){
					n = r_dimensions.z;
				}
				else{
					n = r_dimensions.y;
				}
				
				localPosition.z = n * 1.5f;
				
				localPosition.y = n * 1.5f;
				
				cameraNode.transform.localPosition = localPosition;
			}
			else
			if(c != null && (c_dimensions.x > 0.0f) && (c_dimensions.y > 0.0f)){
				
				Vector3 localPosition = cameraNode.transform.localPosition;
				
				float n = 0.0f;
				
				if(c_dimensions.z > c_dimensions.y){
					n = c_dimensions.z;
				}
				else{
					n = c_dimensions.y;
				}
				
				localPosition.z = n * 1.5f;
				
				localPosition.y = n * 1.5f;
				
				cameraNode.transform.localPosition = localPosition;
			}
			else{
				
				Vector3 localPosition = cameraNode.transform.localPosition;
				
				float n = 1.0f;
				
				localPosition.z = n * 1.5f;
				
				localPosition.y = n * 1.5f;
				
				Debug.Log("Position for default: " + localPosition);
				
				cameraNode.transform.localPosition = localPosition;
				
			}
			
			cameraNode.transform.LookAt(previewObject.transform);
		}
		
		bool GeneratePreview(GameObject previewObject){
		
			//This is the meat of the business
			//And will also be called repeatedly
			//As there is a frame-sync thing 
			//With ... creating screencaps and such
						
			if(previewObject == null){
				return true;
			}
			
			if(previewObjects.ContainsKey(previewObject.name)){
				
				pendingTextureImports.Add(previewObject.name);
			
				//Delete it
				string name = previewObject.name;
				
				Editor.DestroyImmediate(previewObjects[name].gameObject);
								
				TextureImporter importer = AssetImporter.GetAtPath("Assets/"+sourceDirectory+"/"+name+".png") as TextureImporter;
				
				if(importer == null){
					
					AssetDatabase.Refresh();
					
					return false;					
				}
				else{
					
					importer.textureType = TextureImporterType.Advanced;
					
					importer.npotScale = TextureImporterNPOTScale.None;
					
					importer.mipmapEnabled = false;
					
					importer.isReadable = true;
					
					importer.textureFormat = TextureImporterFormat.AutomaticCompressed;
					
					AssetDatabase.ImportAsset("Assets/"+sourceDirectory+"/"+name+".png", ImportAssetOptions.ForceUpdate);
				}
				
				previewObjects.Remove(name);
								
				AssetDatabase.Refresh();
				
				return true;
			}
			else{
							
				EditorApplication.ExecuteMenuItem("Window/Game");
				
				allLightingSettings = new bool[SceneView.sceneViews.Count];
				
				for(int i = 0; i < SceneView.sceneViews.Count; i++){
					allLightingSettings[i] = ((SceneView)SceneView.sceneViews[i]).m_SceneLighting;
				}
									
				//Create a new GameObject that will hold the camera and such 
				GameObject previewNode = new GameObject("Preview Node");
				
				GameObject t = GameObject.Instantiate(previewObject) as GameObject;
				t.name = previewObject.name;
				
#if UNITY_4_0
				t.SetActive(true);
#else
				t.SetActiveRecursively(true);
#endif			
				previewObjects.Add(t.name,t);
				
				Camera c = previewNode.AddComponent<Camera>();
				
				Light[] allLights = GameObject.FindObjectsOfType(typeof(Light)) as Light[];
								
				//Now we look at all active cameras in the scene
				//Note them down, and disable the active ones
				//We keep a list of active cameras to re-enable them later
				//This is because some projects may have inactive cameras
				//And it would be mega-annoying if your extension keeps activating their cameras
				List<Camera> activeCameras = new List<Camera>();
				
				Camera[] cams = GameObject.FindObjectsOfType(typeof(Camera)) as Camera[];
				
				for(int i = 0; i < cams.Length; i++){
				
					if(cams[i].enabled){
						activeCameras.Add(cams[i]);
					}
				}
				
				for(int i = 0; i < activeCameras.Count; i++){
										
					activeCameras[i].enabled = false;
				}
				
				
				int cCount = cams.Length;
				
				t.transform.position = new Vector3(1000.0f * cCount + 1000.0f, 1000.0f * cCount + 1000.0f, 1000.0f * cCount + 1000.0f);
				
				c.clearFlags = CameraClearFlags.Color;
				
				c.backgroundColor = Color.black;
				
				c.enabled = true;
				
				c.depth = 1000;
				
				previewNode.transform.parent = t.transform;
				
				if(allLights.Length <= 0){
					Light l = previewNode.AddComponent<Light>();
				
					l.type = LightType.Directional;
					
					l.intensity = 0.5f;
				}
				
				Color ambientColor = RenderSettings.ambientLight;
				
				RenderSettings.ambientLight = Color.white;
				
				//We prefer to use the renderer, because it will factor height and such
				//as opposed to the square itself
				//but we will satisfy ourself with the collider, in the absence of this
				
				PlacePreviewCamera(previewNode,t);
				
				/*float heightOffset = 0.0f;
				float depthOffset = 0.0f;
				
				MeshFilter m = t.GetComponent<MeshFilter>();
				
				Renderer r = t.renderer;
				
				if(r == null){
					r = t.GetComponentInChildren<Renderer>();
				}
				
				if(m != null){
					heightOffset = m.sharedMesh.bounds.size.y * 1.5f;
					depthOffset = (m.sharedMesh.bounds.size.z) * 3.0f;
					
					if(heightOffset == 0.0f){
						heightOffset = -depthOffset;
					}
				}
				else
				if(r != null){
					
					heightOffset = r.bounds.size.y * 1.5f;
					depthOffset = -(r.bounds.size.z) * 3.0f;
					
					if(heightOffset == 0.0f){
						heightOffset = -depthOffset;
					}
					
				}
				else{
					
					Collider col = t.collider;
					
					if(col == null){
						col = t.GetComponentInChildren<Collider>();
					}
					
					if(col == null){
					}
					else{
						heightOffset = col.bounds.size.y * 1.5f;
						depthOffset = -(col.bounds.size.z) * 3.0f;
						
						if(heightOffset == 0.0f){
							heightOffset = -depthOffset;
						}
					}
					
				}
				
				if(depthOffset == 0.0f){
					
					depthOffset = -heightOffset;
					
				}
				
				if(heightOffset <= 0.1f){
					
					heightOffset = 0.5f * depthOffset;
					
				}
				
				if(depthOffset >= 0.0f){
					
					depthOffset = -1.0f;
					
				}
				
				if(heightOffset <= 0.1f){
					heightOffset = 0.5f * depthOffset;
				}
				
				depthOffset = -depthOffset;
				
				previewNode.transform.up = Vector3.up;
				
				previewNode.transform.localPosition = new Vector3(0.0f,heightOffset,depthOffset);
				
				previewNode.transform.LookAt(t.transform.position);*/
				
				Application.CaptureScreenshot(Application.dataPath + "/" + sourceDirectory + "/" + t.name + ".png");
				
				for(int i = 0; i < activeCameras.Count; i++){
					activeCameras[i].enabled = true;
				}
								
				RenderSettings.ambientLight = ambientColor;
				
				for(int i = 0; i < SceneView.sceneViews.Count; i++){
					((SceneView)SceneView.sceneViews[i]).m_SceneLighting = allLightingSettings[i];
				}
				
				AssetDatabase.Refresh();
				
				return false;
				
			}
		}
		
	}
}
/*using UnityEngine;
using UnityEditor;
using System;
using System.Collections.Generic;

namespace DopplerInteractive.TidyTileMapper.IconManagement{	
	
	public class PreviewDictionary : TextureDictionary{
		
		//We request previews from the editor in sequence
		//They are generated and have a slight pause between being returned
		List<GameObject> previewRequests = new List<GameObject>();
		Dictionary<string,GameObject> previewObjects = new Dictionary<string, GameObject>();
			
		bool[] allLightingSettings = null;
				
		List<Camera> activeCameras = new List<Camera>();
		
		Color ambientColor;
		
		public PreviewDictionary(string sourceDirectory):base(sourceDirectory){}
		
		class PreviewRequest{
			
			public GameObject previewObject;
			public bool overrideExisting = false;
			
			public PreviewRequest(GameObject previewObject, bool overrideExisting){
				this.previewObject = previewObject;
				this.overrideExisting = overrideExisting;
			}
			
		}
		
		public Texture2D GetPreviewTexture(GameObject previewObject){
			
			//Check to see if it already exists
			Texture2D texture = GetTexture(previewObject.name);
			
			if(texture != null){
			
				return texture;
			
			}
				
			//Check to see if it has been created and can be loaded
			LoadTextureForName(previewObject.name);
				
			texture = GetTexture(previewObject.name);	
			
			if(texture != null){
			
				return texture;
			
			}
			
			RequestPreview(previewObject,false);
			
			return null;
			
		}
		
		public Texture2D GetPreviewTexture(GameObject previewObject, bool overwrite){
			
			if(overwrite){
				
				if(previewObjects.ContainsKey(previewObject.name)){
				
					previewObjects.Remove(previewObject.name);
					
				}
				
				RequestPreview(previewObject,overwrite);
				
				return null;
			}
			
			return GetPreviewTexture(previewObject);
			
		}
		
		void RequestPreview(GameObject previewObject,bool overwrite){
				
			if(previewRequests.Contains(previewObject) && !overwrite){
				
				return;
			}
			
			previewRequests.Add(previewObject);
		
		}
				
		public bool UpdatePreviewImports(){
			
			if(IsUpdatingPreviewImport()){
								
				if(GeneratePreview(previewRequests[0])){
					previewRequests.RemoveAt(0);
					return true;
				}
			}
			
			return false;
		}
		
		public bool IsUpdatingPreviewImport(){
			return (previewRequests.Count > 0);
		}
		
		bool GeneratePreview(GameObject previewObject){
		
			//This is the meat of the business
			//And will also be called repeatedly
			//As there is a frame-sync thing 
			//With ... creating screencaps and such
			
			Debug.LogWarning("Generating preview");
			
			if(previewObject == null){
				return true;
			}
			
			if(previewObjects.ContainsKey(previewObject.name)){
				
				Application.CaptureScreenshot(Application.dataPath + "/" + sourceDirectory + "/" + previewObject.name + ".png");
			
				RenderSettings.ambientLight = ambientColor;
				
				
				for(int i = 0; i < SceneView.sceneViews.Count; i++){
					((SceneView)SceneView.sceneViews[i]).m_SceneLighting = allLightingSettings[i];
				}
				
				//Delete it
				string name = previewObject.name;
				
				Editor.DestroyImmediate(previewObjects[name].gameObject);
				
				for(int i = 0; i < activeCameras.Count; i++){
					activeCameras[i].enabled = true;
				}
				
				TextureImporter importer = AssetImporter.GetAtPath("Assets/"+sourceDirectory+"/"+name+".png") as TextureImporter;
				
				if(importer == null){
					Debug.LogWarning("No Texture Importer found at : " + "Assets/"+sourceDirectory+"/"+name+".png");
				}
				else{
					importer.textureType = TextureImporterType.Advanced;
					
					importer.npotScale = TextureImporterNPOTScale.None;
					
					importer.mipmapEnabled = false;
					
					importer.isReadable = true;
				}
				
				previewObjects.Remove(name);
					
				AssetDatabase.Refresh();
					
				//EditorApplication.ExecuteMenuItem("Window/Scene");
				
				return true;
			}
						
			EditorApplication.ExecuteMenuItem("Window/Game");
			
			allLightingSettings = new bool[SceneView.sceneViews.Count];
			
			for(int i = 0; i < SceneView.sceneViews.Count; i++){
				allLightingSettings[i] = ((SceneView)SceneView.sceneViews[i]).m_SceneLighting;
			}
				
			Debug.Log("Generating preview for " + previewObject.name + ".");
			
			//Create a new GameObject that will hold the camera and such 
			GameObject previewNode = new GameObject("Preview Node");
			
			GameObject t = GameObject.Instantiate(previewObject) as GameObject;
			t.name = previewObject.name;
			t.SetActiveRecursively(true);
			
			Camera[] cameras = GameObject.FindObjectsOfType(typeof(Camera)) as Camera[];
						
			previewObjects.Add(t.name,t);
			
			Camera c = previewNode.AddComponent<Camera>();
			
			int cCount = cameras.Length;
			
			t.transform.position = new Vector3(1000.0f * cCount, 1000.0f * cCount, 1000.0f * cCount);
			
			//Now we look at all active cameras in the scene
			//Note them down, and disable the active ones
			//We keep a list of active cameras to re-enable them later
			//This is because some projects may have inactive cameras
			//And it would be mega-annoying if your extension keeps activating their cameras
						
			activeCameras = new List<Camera>();
			
			if(cameras != null){
								
				for(int i = 0; i < cameras.Length; i++){
				
					if(cameras[i].enabled){
						activeCameras.Add(cameras[i]);
					}
				}
				
				for(int i = 0; i < activeCameras.Count; i++){
					activeCameras[i].enabled = false;
					activeCameras[i].RenderDontRestore();
				}
			}
				
			c.clearFlags = CameraClearFlags.SolidColor;
			
			c.backgroundColor = Color.black;
			
			c.enabled = true;
			
			c.depth = 1000.0f;
						
			previewNode.transform.parent = t.transform;
			
			Light l = previewNode.AddComponent<Light>();
			
			l.type = LightType.Directional;
			
			ambientColor = RenderSettings.ambientLight;
			
			RenderSettings.ambientLight = Color.white;
			
			c.RenderDontRestore();
			
			//We prefer to use the renderer, because it will factor height and such
			//as opposed to the square itself
			//but we will satisfy ourself with the collider, in the absence of this
			if(t.renderer != null){
				previewNode.transform.localPosition = new Vector3(0.0f,t.renderer.bounds.size.y*1.5f,-t.renderer.bounds.size.z * 3.0f);
			}
			else{
				previewNode.transform.localPosition = new Vector3(0.0f,t.collider.bounds.size.y*1.5f,-t.collider.bounds.size.z * 3.0f);
			}
			
			previewNode.transform.LookAt(t.transform.position);
						
			Debug.Log("Saving preview to: " + Application.dataPath + "/" + sourceDirectory + "/" + t.name + ".png");
						
			SceneView.RepaintAll();
									
			return false;
		}
		
	}
}*/