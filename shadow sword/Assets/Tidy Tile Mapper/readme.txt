Hello! Congratulations on your purchase of a shiny new Tidy Tile-Mapper!

For assistance with using Tidy Tile-Mapper, or to share your feelings 
and opinions about the future of Tidy Tile-Mapper, please visit:
http://tools.dopplerinteractive.com

For support and that personal touch, contact:
support@dopplerinteractive.com

But above all - enjoy!

Update: Version 1.4.6 - 13th March, 2013
Bug Fixes:
-Updated StreamingMapColorProfile to use new Streaming Node system (a missing: [Serializable] tag caused Null Reference exceptions).

Update: Version 1.4.5 - 8th February, 2013

Bug Fixes:
-Fixed mistaken Block link in Player Controller block

Update: Version 1.4.3.3 - 2nd January, 2013

Features:
-Added a new Streaming Maps example - "StreamingWorld". This shows the digging example that you see in the Tidy TileMapper videos. Enjoy!
-Added a tonne of new orientations to the Block Editor. This is 100% the good deed of Alex Evangelou of Angry Fish Studios (http://www.angryfishstudios.com/)
He created the icons, and didn't even get audibly angry with me when I repeatedly, frustratingly forgot to add them to the tool. He has my love, he should get yours too.
-Added Streaming Map Node function - now blocks, backgrounds and variants can be streamed from one handy data construct
-Improved runtime background painting.

Bug Fixes:
-Fixed issue with AssetPool.Destroy() not properly disabling objects (victim of a hurried copy/paste)
-Fixed issue with background painting generally being broken

Notes for those updating from DLL to Full Source version:

Hello friends! Unity 4.0 is here, and spring is in the air (figuratively)
In the spirit of free love, bees and flowers I have modified this tool to be
full source (as opposed to DLL).
NOTE PLEASE: IMPORTANT: If you are converting from an earlier version of TTM
You must do the following things:
1) Delete the Editor\\Editor Logic\\TidyTileMapper_Editor.dll file
2) Delete the Mapping\\TidyTileMapper.dll file
3) Fret momentarily - because all of your prefabs have lost their scripts!
4) Go to Window>Find and fix missing scripts (This script was borrowed from unitygems.com)
5) Done!
Note this script was found online - I have not modified it in any way,
and have retained the folder naming that points to the creator's website.
Give them some love.

With much platonic love,
Joshua McGrath.