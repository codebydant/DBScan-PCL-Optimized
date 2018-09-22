#include <GL/glut.h>
#include <iostream>

using namespace std;

namespace mouseUtils
{
    bool mouseLeftDown;
    bool mouseRightDown;
    float mouseX, mouseY;
    float cameraAngleX;
    float cameraAngleY;
    float cameraDistance;

    // Callbacks
    void mouse(int button, int state, int x, int y)
    {
        mouseX = x;
        mouseY = y;

        if(button == GLUT_LEFT_BUTTON)
        {
            if(state == GLUT_DOWN)
            {
                mouseLeftDown = true;
            }
            else if(state == GLUT_UP)
                mouseLeftDown = false;
        }

        else if(button == GLUT_RIGHT_BUTTON)
        {
            if(state == GLUT_DOWN)
            {
                mouseRightDown = true;
            }
            else if(state == GLUT_UP)
                mouseRightDown = false;
        }
    }


    void mouseMotion(int x, int y)
    {
        if(mouseLeftDown)
        {
            cameraAngleY += (x - mouseX);
            cameraAngleX += (y - mouseY);
            mouseX = x;
            mouseY = y;
        }
        if(mouseRightDown)
        {
            cameraDistance -= (y - mouseY) * 0.2f;
            mouseY = y;
        }
    }

    // Initialization
    void init(float CAMERA_DISTANCE)
    {
        mouseLeftDown = mouseRightDown = false;
        mouseX = mouseY = 0;

        cameraAngleX = cameraAngleY = 0;
        cameraDistance = CAMERA_DISTANCE;
    }

    // Mouse Transformation
    void applyMouseTransform()
    {
        glTranslatef(0, 0, -cameraDistance);
        glRotatef(cameraAngleX, 1, 0, 0);
        glRotatef(cameraAngleY, 0, 1, 0);
    }

        // Mouse transformation from pivot point
    void applyMouseTransform(float x, float y, float z)
    {
        glTranslatef(x, y, z);
        glTranslatef(0, 0, -cameraDistance);
        glRotatef(cameraAngleX, 1, 0, 0);
        glRotatef(cameraAngleY, 0, 1, 0);
        glTranslatef(-x, -y, -z);
    }
}


