#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t offset, int direction)
{
    (void)file;
    (void)offset;
    (void)direction;
    return 0;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = EINVAL;
    return -1;
}

int _write(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    return length;
}
