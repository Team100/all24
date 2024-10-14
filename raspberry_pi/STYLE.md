# Python Style

## Imports
Python offers two import styles:

- import x
- from x import y

Vscode sometimes seems unable to understand the former style, so we should prefer the latter.

We should always use the package name for x, even though sometimes it would work to use a dot (i.e. for example in the same package).

<https://stackoverflow.com/questions/710551/use-import-module-or-from-module-import>

https://google.github.io/styleguide/pyguide.html

https://stackoverflow.com/questions/47319423/import-a-module-from-both-within-same-package-and-from-outside-the-package-in-py


# \_\_init\_\_.py

It seems not necessary to sprinkle \_\_init\_\_.py files in the app code, but it does seem necessary to do so in the test code, so that unittest.TestLoader, and vscode's extension, can find the tests.

# Packages

As much as possible, the sub-packages within the "app" top-level package should mirror the packages used in the RoboRIO Java code.  So, for example, the RoboRIO "Identity" concept lives in the "config" package, so it does here too.

# Protocols

This project makes extensive use of python Protocols, which are like Java interfaces.

To learn about Protocols for interface specification, see
https://typing.readthedocs.io/en/latest/spec/protocol.html