using UnityEngine;
using System.Collections;

namespace DopplerInteractive.TidyTileMapper.Words{
	public class TidyMessages {
		
		//This class simply holds many many many strings
		//for outputting error mesages
		
		public static string MAP_CREATOR_PATH_ALLOW_DIAGONALS = "Diagonals?";
		
		public static string MAP_CREATOR_PATH_RANDOMISATION = "Randomise?";
		public static string MAP_CREATOR_PATH_WIDTH = "Path Width:";
				
		public static string MAP_CREATOR_DRAW_PATH = "Draw Path";
		
		public static string BLOCK_EDITOR_CLOSE_DIALOG = "Closing block editor...";
		public static string BLOCK_EDITOR_CLOSE_PROMPT = "Save changes to your current block?";
		public static string BLOCK_EDITOR_CLOSE_CONFIRM = "Yes";
		public static string BLOCK_EDITOR_CLOSE_REJECT = "No";
		
		public static string BLOCK_EDITOR_SWITCH_DIALOG = "Switching blocks...";
		public static string BLOCK_EDITOR_SWITCH_PROMPT = "Save changes to your current block?";
		public static string BLOCK_EDITOR_SWITCH_CONFIRM = "Yes";
		public static string BLOCK_EDITOR_SWITCH_REJECT = "No";
		
		public static string BLOCK_EDITOR_X_OFFSET = "X Offset:";
		public static string BLOCK_EDITOR_Y_OFFSET = "Y Offset:";
		public static string BLOCK_EDITOR_Z_OFFSET = "Z Offset:";
		
		public static string BLOCK_EDITOR_X_ROTATION = "X Rotation:";
		public static string BLOCK_EDITOR_X_ROTATION_TOOLTIP = "All internal members of this block will be rotated on the x-axis by this amount.";
		public static string BLOCK_EDITOR_Y_ROTATION = "Y Rotation:";
		public static string BLOCK_EDITOR_Y_ROTATION_TOOLTIP = "All internal members of this block will be rotated on the y-axis by this amount.";
		public static string BLOCK_EDITOR_Z_ROTATION = "Z Rotation:";
		public static string BLOCK_EDITOR_Z_ROTATION_TOOLTIP = "All internal members of this block will be rotated on the z-axis by this amount.";
		
		
		public static string BLOCK_EDITOR_X_OFFSET_TOOLTIP = "All internal members of this block will be offset on the x-axis by this amount.";
		public static string BLOCK_EDITOR_Y_OFFSET_TOOLTIP = "All internal members of this block will be offset on the y-axis by this amount.";
		public static string BLOCK_EDITOR_Z_OFFSET_TOOLTIP = "All internal members of this block will be offset on the z-axis by this amount.";
		
		public static string BLOCK_EDITOR_BLOCKLIST_TITLE = "Your Blocks:";
		
		public static string BLOCK_EDITOR_WINDOW_NAME = "Tidy Block Editor";
		
		public static string BLOCK_EDITOR_ADD_BLOCK = "Add Block";
		
		public static string BLOCK_EDITOR_BLOCK_NAME = "Block Name:";
		
		public static string BLOCK_EDITOR_NO_PLAY = "You must close the Block Editor before pressing 'Play'.";
		public static string BLOCK_EDITOR_SAVE_BUTTON = "Save Changes";
		
		public static string BLOCK_EDITOR_ADD_BLOCK_TOOLTIP = "Add a new block to your palette";
		public static string BLOCK_EDITOR_SAVE_BUTTON_TOOLTIP = "Save your block and update its prefab";
		public static string BLOCK_EDITOR_ACT_AS_EMPTY_BLOCK_TOOLTIP = "This block will be treated as empty when deciding on the orientation of surrounding blocks.";
		public static string BLOCK_EDITOR_RETAIN_COLLIDER_TOOLTIP = "If enabled, this block will retain a solid collider. If disabled, this block will instead retain a trigger-collider.";
		public static string BLOCK_EDITOR_BLOCKSET_IS_DEFAULT_TOOLTIP = "When enabled, this set of objects will be used as the Default set for this block.";
		public static string BLOCK_EDITOR_BLOCKSET_ADD_VARIANT = "Add Variant";
		public static string BLOCK_EDITOR_BLOCKSET_ADD_VARIANT_TOOLTIP = "Add a variant object to this block-set - used when painting random, or cycling through variants.";
		public static string BLOCK_EDITOR_BLOCKSET_REMOVE_VARIANT_TOOLTIP = "Remove this variant from the set.";
		public static string MAP_CREATOR_HELP_TOOLTIP = "View Online help";
		public static string MAP_CREATOR_ABOUT_TOOLTIP = "About us - who we are, what we do.";
		public static string BLOCK_EDITOR_BLOCK_OPTIONS = "Block Options";
		public static string BLOCK_EDITOR_ACT_AS_EMPTY_BLOCK = "Act empty?";
		public static string BLOCK_EDITOR_RETAIN_COLLIDER = "Keep Collider?";
		
		public static string BLOCK_EDITOR_BASIC_BLOCKS = "Basic Blocks";
		public static string BLOCK_EDITOR_FLAT_SURFACES = "Flat Surfaces";
		public static string BLOCK_EDITOR_BLOCK_LINES = "Lines";
		public static string BLOCK_EDITOR_PROTRUDING_BLOCKS = "Protruding Blocks";
		public static string BLOCK_EDITOR_CORNERS = "Outer Corners";
		public static string BLOCK_EDITOR_INNER_CORNERS = "Inner Corners";
		public static string BLOCK_EDITOR_SLOPED_EDGES = "Sloped Edges";
		public static string BLOCK_EDITOR_INTERSECTIONS = "Intersections";
		
		public static string MAP_CREATOR_VISIBILITY_HEADER = "Block-Map Layer Visibility";
		public static string MAP_CREATOR_LAYER_VISIBILITY_SUFFIX = " visibility.";
		public static string MAP_CREATOR_LAYER_VISIBILITY_PREFIX = "Map: ";
		
		public static string MAP_CREATOR_NAME_EXISTS = "Map name already exists in scene.";
		
		public static string MAP_CREATOR_AUTOMATED_GENERATION_FOLDOUT = "Maze Generation";
		public static string MAP_CREATOR_MAZE_GENERATION_LABEL = "Maze Generation";
		public static string MAP_CREATOR_MAZE_TYPE_LABEL = "Maze Type:";
		public static string MAP_CREATOR_GENERATE_MAZE = "Generate Maze";
		
		public static string MAP_CREATOR_STRIPPING_LEVEL = "Stripping Level:";
		
		public static string MAP_CREATOR_LEVEL_GENERATION_WIDTH = "Width (Cells):";
		public static string MAP_CREATOR_LEVEL_GENERATION_HEIGHT = "Height (Cells):";
		
		public static string MAP_CREATOR_WINDOW_NAME = "Tidy TileMapper";
		public static string MAP_CREATOR_DEFAULT_MAP_MESSAGE = "Status: Map creation inactive.";
		public static string MAP_CREATOR_BLOCK_EDITOR = "Block Editor";
		public static string MAP_CREATOR_BLOCK_LIST = "Your Blocks:";
		public static string MAP_CREATOR_NEW_MAP_NAME = "New Map Name:";
		public static string MAP_CREATOR_ADD_MAP = "Add Map";
		public static string MAP_CREATOR_NO_BLOCK_CHOSEN = "Select a block below...";
		public static string MAP_CREATOR_NO_NAME = "Enter a name for the new map.";
		public static string MAP_CREATOR_MAP_CREATED = "Status: Map creation successful!";
		public static string MAP_CREATOR_TOOL_SELECTION = "Map tools";
		public static string MAP_CREATOR_INACTIVE = "Tile Mapping inactive.";
		public static string MAP_CREATOR_CYCLING_ACTIVE = "Tile Mapping active: Cycling Blocks.";
		public static string MAP_CREATOR_PAINTING_ACTIVE = "Tile Mapping active: Painting Blocks.";
		public static string MAP_CREATOR_DEFAULT_BLOCK_NAME = "None.";
		public static string MAP_CREATOR_DEFAULT_BLOCK_COORDS = "None.";
		public static string MAP_CREATOR_DEFAULT_BLOCK_ORIENTATION = "None.";
		public static string MAP_CREATOR_BLOCK_NAME_PREFIX = "Block Name: ";
		public static string MAP_CREATOR_BLOCK_COORDS_PREFIX = "Coordinates: ";
		public static string MAP_CREATOR_BLOCK_ORIENTATION_PREFIX = "Orientation: ";
		public static string MAP_CREATOR_BLOCK_CATEGORY = "Block Management:";
		public static string MAP_CREATOR_NO_MAP_SELECTED = "None.";
		public static string MAP_CREATOR_SELECTED_MAP_LABEL = "Selected Map: ";
				
		public static string MAP_CREATOR_NO_MAPS_EXIST = "No block-maps exist in scene.";
		
		public static string MAP_CREATOR_CREATE_MAP_CATEGORY = "Map Creation:";
		public static string MAP_CREATOR_CURRENT_MAP_CATEGORY = "Map Management:";
		
		public static string MAP_CREATOR_NO_CHUNK_ERROR = "No chunk prefab found.\nCannot create maps.";
		
		public static string MAP_CREATOR_PUBLISH_MAP = "Publish Map.";
		
		public static string MAP_CREATOR_CHUNK_WIDTH = "Map Width: ";
		public static string MAP_CREATOR_CHUNK_WIDTH_SUFFIX = " chunks.";
		public static string MAP_CREATOR_CHUNK_HEIGHT = "Map Height: ";
		public static string MAP_CREATOR_CHUNK_HEIGHT_SUFFIX = " chunks.";
		
		public static string MAP_CREATOR_NEW_MAP_CHUNK_WIDTH = "Chunk Width: ";
		public static string MAP_CREATOR_NEW_MAP_CHUNK_HEIGHT = "Chunk Height: ";
		
		public static string MAP_CREATOR_MAP_FUNCTIONS_DISABLED = "Map-specific functions are\ndisabled while editing.";
		
		public static string MAP_CREATOR_ADVANCED_MAP_OPTIONS = "Advanced";
		
		public static string MAP_CREATOR_TILE_WIDTH = "Tile Width:";
		public static string MAP_CREATOR_TILE_HEIGHT = "Tile Height:";
		public static string MAP_CREATOR_TILE_DEPTH = "Tile Depth: ";
		public static string MAP_CREATOR_MAP_GROW_AXIS = "Growth Axis:";
		
		public static string MAP_CREATOR_TIDY_ADVANCED_OPTIONS = "Tidy TileMapper: More Options";
		
		public static string MAP_CREATOR_HELP = "Online Help";
		public static string MAP_CREATOR_ABOUT = "About Us";
		public static string MAP_CREATOR_CLEAR_PREVIEW_IMAGES = "Clear Preview Images";
		public static string MAP_CREATOR_SUBMIT_BUG = "Report-A-Bug";
		
		public static string MAP_CREATOR_CYCLE_OPTIONS = "Block-Cycle Options";
		public static string MAP_CREATOR_PAINT_OPTIONS = "Block-Paint Options";
		
		public static string MAP_CREATOR_PAINT_RANDOM = "Randomise?";
		
		public static string MAP_CREATOR_PUBLISH_MAP_EXISTS_TITLE = "A prefab exists with this name!";
		public static string MAP_CREATOR_PUBLISH_MAP_EXISTS_INFO = "Do you want to over-write it?";
		public static string MAP_CREATOR_PUBLISH_MAP_EXISTS_OVERWRITE = "Yes";
		public static string MAP_CREATOR_PUBLISH_MAP_EXISTS_DONT_OVERWRITE = "No";
		
		public static string MAP_CREATOR_DEFAULT_PUBLISH_MESSAGE = "Status: No publish attempted.";
		
		public static string MAP_CREATOR_BLOCK_MOVE_ROUNDNESS = "Roundness";
		public static string MAP_CREATOR_BLOCK_MOVE_OPTIONS = "Block Move Options";
		public static string MAP_CREATOR_BLOCK_MOVE_RADIUS_TYPE = "Radius Type";
		public static string MAP_CREATOR_BLOCK_MOVE_RADIUS = "Radius";
		
		public static string MAP_CREATOR_BLOCK_MOVE_ACTIVE = "Tile Mapping active: Moving Blocks.";
		
		public static string PREVIEW_GENERATING_PREVIEWS_TITLE = "Generating previews...";
		public static string PREVIEW_GENERATING_PREVIEWS_INFO = "A minor inconvenience, but worth it.";
		
		public static string PUBLISH_UTILITY_SUCCESS_AT = "Success! Created map prefab at:\n";
		public static string PUBLISH_UTILITY_FAILURE = "Failed to create map prefab.";
		
		public static string PUBLISH_UTILITY_BUILDING = "Building map - please be patient\nit may take a while.";
		
		public static string PUBLISH_UTILITY_NOTIFICATION = "Building map...";
		
		public static string MAP_CREATOR_STREAMING_MAPS = "Streaming Maps";
		
		public static string MAP_CREATOR_ADD_STREAMING_MAP = "New Streaming Map";
	}
}
