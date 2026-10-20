/* Inject serial write failures only on the test's pseudo-terminal slave.
 * The bridge emulator, stdout/stderr and physical devices are not intercepted. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static atomic_uint attempts = 0;

unsigned int rebot_test_serial_write_attempts(void) { return atomic_load(&attempts); }

ssize_t write(int fd, const void *buffer, size_t count) {
  ssize_t (*real_write)(int, const void *, size_t) = dlsym(RTLD_NEXT, "write");
  const char *device = getenv("REBOT_TEST_SERIAL_DEVICE");
  const char *failure = getenv("REBOT_TEST_SERIAL_FAILURE");
  char name[256];
  if (!device || !failure || ttyname_r(fd, name, sizeof(name)) || strcmp(name, device))
    return real_write(fd, buffer, count);
  const unsigned int attempt = atomic_fetch_add(&attempts, 1);
  if (!strcmp(failure, "zero")) return 0;
  if (!strcmp(failure, "disconnected")) { errno = EIO; return -1; }
  if (!strcmp(failure, "persistent")) { errno = EAGAIN; return -1; }
  if (!strcmp(failure, "transient")) {
    if (attempt == 0 || attempt == 1 || attempt == 3) {
      errno = attempt == 1 ? EINTR : EAGAIN;
      return -1;
    }
    if (attempt == 2 && count > 7) return real_write(fd, buffer, 7);
  }
  return real_write(fd, buffer, count);
}
