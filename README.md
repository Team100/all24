# all24

all24 contains *all* the Team 100 code for *2024.*  There's just one repository,
so that sharing library code across projects is easier.

This style of code management is sometimes called a ["monorepo"](https://en.wikipedia.org/wiki/Monorepo) if you want to read more about it.

Here is the directory layout:

* **lib**: evergreen library code
* **comp**: competition code for 2024
  * **swerve100**: roborio code
  * **vision**: raspberry pi code
* **studies**: small independent projects
  * your project
  * another project
  * etc ...

# VS Code

When you open the swerve100 project, VS Code should show a message in the bottom-right corner suggesting that you open the "workspace" instead, which includes lib:

<img src="openworkspace.png" width=350/>

When you have VS Code correctly set up, the Explorer view should contain two top-level folders: "swerve100" and "lib":

<img src="workspace.png" width=350/>

# Setup

To include "lib" in your own "study" workspace, you need to do two things:

* Add a workspace file, to make vscode features work with lib code
* Edit build.gradle, so gradle builds include lib code
* Copy the lib vendordeps to your project.

## Workspace

To make a workspace file, in your project in vscode, click "add folder to workspace" choosing the lib folder, and then "save workspace to file," choosing your project directory as the location to save it.

This should result in a file at the root level of your project (next to build.gradle) that looks like this:

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

## Build.gradle

In your build.gradle file, add this clause just after the "plugins" clause (near the top).

```gradle
sourceSets {
    main {
        java {
            srcDir "../../lib/src/main/java"
        }
    }
}
```

You also need to add some dependencies.  So inside the "dependencies" section, add these:

```gradle
    implementation 'org.msgpack:jackson-dataformat-msgpack:0.9.3'
    implementation "org.ejml:ejml-simple:0.43.1"
```

## Vendordeps

To copy the vendordeps files, find the vendordeps directory in lib, highlight one, say, "NavX.json", and click "copy."

Then find the vendordeps directory in your project (it should contain WPILibNewCommands.json and nothing else).

Highlight the directory name ("vendordeps") and click "paste."

Repeat this process for each of the other two vendordeps files.
