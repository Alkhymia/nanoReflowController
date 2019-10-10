import os

Import("env")

if not os.path.exists(env.subst("$PROJECT_DIR/lib/PDQ_GFX_Libs")):
    print "Cloning PDQ_GFX repo ... "
    env.Execute("git clone --depth 100 https://github.com/XarkLabs/PDQ_GFX_Libs $PROJECT_DIR/lib/PDQ_GFX_Libs")

if not os.path.exists(env.subst("$PROJECT_DIR/lib/PID_Autotune")):
    print "Cloning PID_Autotune repo ... "
    env.Execute("git clone --depth 100 https://github.com/br3ttb/Arduino-PID-AutoTune-Library $PROJECT_DIR/lib/PID_Autotune")
    