/*
    Modified from
    https://stackoverflow.com/a/2754563/1633026
    https://stackoverflow.com/a/20706268/1633026

    compile with: -framework CoreFoundation -framework IOKit

*/

#include <IOKit/IOKitLib.h>

bool get_platform_uuid(char *buf, int bufSize)
{
    io_registry_entry_t ioRegistryRoot = IORegistryEntryFromPath(kIOMainPortDefault, "IOService:/");
    CFStringRef uuidCf = (CFStringRef)IORegistryEntryCreateCFProperty(ioRegistryRoot, CFSTR(kIOPlatformUUIDKey), kCFAllocatorDefault, 0);
    IOObjectRelease(ioRegistryRoot);
    bool success = CFStringGetCString(uuidCf, buf, bufSize, kCFStringEncodingMacRoman);
    CFRelease(uuidCf);
    return success;
}
