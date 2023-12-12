#include <filesystem.h>
#include <root_directory.h>


std::string FileSystem::get_absolute_path(void)
{
    static char const * env_root = getenv("LOGL_ROOT_PATH");
    static char const * given_root = (env_root != nullptr ? env_root : logl_root);
    static std::string root  = (given_root != nullptr ? given_root : "");
    return root;
}

