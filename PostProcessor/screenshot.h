#ifndef _SCREENSHOT_H_
#define _SCREENSHOT_H_

// this function must be called after window has been rendered
void screenshot(vtkRenderWindow *render_win, const char *file_name);

#endif