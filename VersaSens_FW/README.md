# VersaSens_FW




## How to add your own application?
The firmware of VersaSens provides a predefined folder (`src/user_app/`) to easilly add a user application.

To add an application, few steps are required:
- **Add your code:** the application should be addedd in the `user_app/` folder. Specifically, this contains two subfolders:
    - `user_app/src/`: this should contain all the source files of the application
    - `user_app/inc/`: this should contain all the header files of the application

- **Include the application:** the header that declares the main thread of the app should be included in the main module of the FW.

- **Init app thread init:** after the initial configuration of VersaSens done in the main module, the application thread can be initialized simply by calling the `init_user_app_thread(user_app_thread, priority);` function of the `start_app_api` module. This module contains all the thread specific variables and includes to properly initialize the thread. After this, the thread will start and will be executed based on its priority.

#### Extra: thread stack
Each thread has a stack size. Specifically, the user application has 1024 bytes by deffault. However, if this is not enough, it can be easilly changed in the `CMakeLists.txt` file by modifying the corresponding value of `USER_APP_STACK_SIZE` definition.

#### Example
A simple example is already implemented and provided in the `test_user_app` test file (path: `src/testbench/test_user_app.c`). This will launch a simple HelloWorld-like application already provided inside the `user_app/` folder. In order to test it, it's necessary to modify the target sources of the CMakeFiles.txt to point to the correct main file, as follows:

```
target_sources(app PRIVATE 
    src/testbench/test_user_app.c 
```