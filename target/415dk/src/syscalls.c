#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <sys/stat.h>

extern int errno;
static char * stack_ptr __asm("sp");

__attribute__((weak)) int _write(int file, char *ptr, int len) {
    extern int uart1_write(const char *buf, int len);
    if(file != 1){
        return 0;
    }
	return uart1_write(ptr, len);;
}

__attribute__((weak)) int _read(int file, char *ptr, int len) {(void)file; (void)ptr; (void)len; return 0; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig){(void)pid; (void)sig; errno = EINVAL; return -1; }
int _close(int file) {(void)file; return -1;}
int _fstat(int file, struct stat *st) {(void)file;	st->st_mode = S_IFCHR; return 0; }
int _isatty(int file){ (void)file; return 1; }
int _lseek(int file, int ptr, int dir){(void)file; (void)ptr; (void)dir; return 0; }
int _open(char *path, int flags, ...) {	/* Pretend like we always fail */ (void)path; (void)flags; return -1; }

char *_sbrk(int incr)
{
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
		errno = ENOMEM;
		return (char *) -1;
	}

	heap_end += incr;

	return (char *) prev_heap_end;
}
