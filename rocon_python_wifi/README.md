## Overview

This has been brought into the ros package system since it's not available through anything outside of pypi
which can be awkward when ensuring dependencies (of the correct version) are installed.

Some important points:

* *pythonwifi->rocon_python_wifi* : to avoid possible conflicts if someone happens to have pythonwifi installed from pypi
* *Version 0.5.0* : they're working on a new api for newer versions, expect this to change sometime in the future.
