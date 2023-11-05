# Team 100 Library

To use this, you will need to make two changes to your vscode setup.

* in vscode, click "new project" two times, to make "lib" and "app" projects in the monorepo as siblings.
* in the "app" project, click "add folder to workspace", and add the "lib" folder.
* click "save workspace as..." and save the workspace file inside the "app" folder.
* in the "app" build.gradle, add the sourceSets clause below

That's it!  you'll be able to open the "lib" folder like a normal project, and when you try to open
the "app" folder, vscode will invite you to open the multi-root workspace instead.  Everything works (e.g .autocomplete, hints etc).

so the layout should be like this:

* all24
  * comp
    * swerve100
  * lib 
  * studies
    * some study
    * another study
    * ...

The build.gradle change for swerve100 is this:

```gradle
sourceSets {
    main {
        java {
            srcDir "../../lib/src/main/java"
        }
    }
}
```

The workspace file in swerve100 should look like this:

```json
{
	"folders": [
		{
			"path": "."
		},
		{
			"path": "../../lib"
		}
	],
    "settings": {
        "java.configuration.updateBuildConfiguration": "automatic",
        "java.server.launchMode": "Standard"
    }
}
```
