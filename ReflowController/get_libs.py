import os

Import("env")

if not os.path.exists(env.subst("$PROJECT_DIR/lib/PDQ_GFX_Libs")):
    print("Cloning PDQ_GFX repo ... ")
    env.Execute("git clone --depth 100 https://github.com/XarkLabs/PDQ_GFX_Libs $PROJECT_DIR/lib/PDQ_GFX_Libs")
    