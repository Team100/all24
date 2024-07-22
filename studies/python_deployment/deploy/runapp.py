import sys
sys.path.insert(0, "app.zip")
print(sys.path[0])

from app import foo
from app import bar

foo.doFoo()
bar.doBar()
