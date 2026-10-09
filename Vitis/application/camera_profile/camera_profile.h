#ifndef CAMERA_PROFILE_TASK_H
#define CAMERA_PROFILE_TASK_H

#define COLOR_PROFILE_DEFAULT   2U

extern unsigned int color_profile;
extern unsigned int camera_history_count;

void CameraNextColorProfile(void);
void CameraSetColorProfile(unsigned int index);
void CameraUndoColorProfile(void);

#endif /* CAMERA_PROFILE_TASK_H */
