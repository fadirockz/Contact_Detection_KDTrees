#include<iostream>
#include<fstream>
#include<time.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <math.h>
using namespace std;
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define M_PI 3.14159265358979323846

#define NUM_OF_CELLS 250
int cellRadius = 20;


const int k = 3;
int visited;
GLFWwindow *window;
GLfloat *circleVerticesX;
GLfloat *circleVerticesY;
GLfloat *circleVerticesZ;
GLfloat *allCircleVertices;
double resolved[6] = { 0 };
GLfloat analysis[100000][2] = { {} };
GLfloat pointVertex[] = { SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 };
double totalTime = 0;

struct Node
{
	double point[k]; // To store k dimensional point
	Node *left, *right;
};
double points[10000][3] = { {} };
struct Node *root, *found;

struct Node* newNode(double arr[])
{
	struct Node* temp = new Node;

	for (int i = 0; i<k; i++)
		temp->point[i] = arr[i];

	temp->left = temp->right = NULL;
	return temp;
}

Node *insertRec(Node *root, double point[], unsigned depth)
{
	if (root == NULL)
		return newNode(point);

	unsigned cd = depth % k;

	if (point[cd] < (root->point[cd]))
		root->left = insertRec(root->left, point, depth + 1);
	else
		root->right = insertRec(root->right, point, depth + 1);

	return root;
}
Node* insert(Node *root, double point[])
{
	return insertRec(root, point, 0);
}

void print_tree(Node *my_tree) {

	if (my_tree == NULL)
	{
		return;
	}
	if (my_tree->left) print_tree(my_tree->left);
	cout << my_tree->point[0] << "," << my_tree->point[1] << " and id = " << my_tree->point[2] << endl;
	if (my_tree->right) print_tree(my_tree->right);
}
double OverlapArea(double p1[], double p2[])
{
	double x1 = p1[0];
	double x2 = p2[0];
	double y1 = p1[1];
	double y2 = p2[1];
	double X = x1 - x2;
	double Y = y1 - y2;
	double D;
	D = X*X + Y*Y;
	D = sqrt(D);
	double inv = acos(D / (2 * cellRadius));
	double A = cellRadius*cellRadius* inv - (D / 4)*(sqrt(4 * cellRadius*cellRadius - D*D));
	return A;

}
// Driver program to test above functions
void deleteTree(struct Node* node)
{
	if (node == NULL) return;

	deleteTree(node->left);
	deleteTree(node->right);

	free(node);
}

double
dist(Node *a, Node *b, int dim)
{
	double d = (a->point[0] - b->point[0])*(a->point[0] - b->point[0]) + (a->point[1] - b->point[1])*(a->point[1] - b->point[1]);
	return d;
}
void nearest(Node *root, Node *nd, int i, int dim,
	Node **best, double *best_dist)
{
	double d, dx, dx2;

	if (!root) return;
	d = dist(root, nd, dim);
	dx = root->point[i] - nd->point[i];
	dx2 = dx * dx;

	visited++;

	if (!*best || d < *best_dist) {
		if (d != 0)
		{
			*best_dist = d;
			*best = root;
		}
	}

	if (!*best_dist) return;

	if (++i >= dim) i = 0;

	nearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist);
	if (dx2 >= *best_dist) return;
	nearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist);
}

int collision_count()
{
	int collision_counter = 0;
	float x1, x2, X, y1, y2, Y, r1, r2, R, D;
	for (int i = 0; i < NUM_OF_CELLS; i++)
	{
		for (int j = i + 1; j < NUM_OF_CELLS; j++)
		{
			x1 = points[i][0];
			x2 = points[j][0];
			X = x1 - x2;

			y1 = points[i][1];
			y2 = points[j][1];
			Y = y1 - y2;

			R = (cellRadius * 2);

			D = X*X + Y*Y;

			if (D < R*R - 0.01)
			{
				collision_counter++;
				break;
			}
		}

	}
	//cout << "Number of collisions are " << collision_counter << endl;
	return collision_counter;
}


double modulus(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool resolve_collisions(Node p1, double p2[])
{
	double t_p1[3];
	double t_p2[3];

	t_p1[0] = p1.point[0];
	t_p1[1] = p1.point[1];
	t_p1[2] = p1.point[2];

	t_p2[0] = p2[0];
	t_p2[1] = p2[1];
	t_p2[2] = p2[2];


	double x1, x2, X, y1, y2, Y, R, D;

	double a = 0.7;

	x1 = p1.point[0];
	y1 = p1.point[1];

	x2 = p2[0];
	y2 = p2[1];

	X = x1 - x2;
	Y = y1 - y2;

	R = (cellRadius + cellRadius);

	D = X*X + Y*Y;

	if (D <= R*R)
	{
		double Fx, Fy;
		Fx = (R - modulus(x1, y1, x2, y2)) / modulus(x1, y1, x2, y2) * a * (x1 - x2);
		Fy = (R - modulus(x1, y1, x2, y2)) / modulus(x1, y1, x2, y2) * a * (y1 - y2);

		t_p1[0] += Fx;
		t_p1[1] += Fy;

		t_p2[0] -= Fx;
		t_p2[1] -= Fy;

		resolved[0] = t_p1[0];
		resolved[1] = t_p1[1];
		resolved[2] = t_p1[2];

		resolved[3] = t_p2[0];
		resolved[4] = t_p2[1];
		resolved[5] = t_p2[2];

		if (resolved[0] < cellRadius)
		{
			resolved[0] = cellRadius;
			//points[i].x += Fx;

		}
		if (resolved[0] > SCREEN_WIDTH - cellRadius)
		{
			resolved[0] = SCREEN_WIDTH - cellRadius;
		}
		if (resolved[1] < cellRadius)
		{
			resolved[1] = cellRadius;
		}

		if (resolved[1] > SCREEN_HEIGHT - cellRadius)
		{
			resolved[1] = SCREEN_HEIGHT - cellRadius;
		}


		if (resolved[3] < cellRadius)
		{
			resolved[3] = cellRadius;
		}
		if (resolved[3] > SCREEN_WIDTH - cellRadius)
		{
			resolved[3] = SCREEN_WIDTH - cellRadius;
		}
		if (resolved[4] < cellRadius)
		{
			resolved[4] = cellRadius;
		}

		if (resolved[4] > SCREEN_HEIGHT - cellRadius)
		{
			resolved[4] = SCREEN_HEIGHT - cellRadius;
		}
		return true;
	}//end of if


	return false;
}
void createRandomPoints()
{
	ofstream fp; // file pointer
	int i = 0;
	int total_spaces_x = 100; // SCREEN_WIDTH / 2;
	int total_spaces_y = 100; // SCREEN_HEIGHT / 2;

	while (i != NUM_OF_CELLS)// number of points
	{

		double rand1, rand2;

		rand1 = cellRadius + (rand() % (SCREEN_WIDTH - 2 * cellRadius));
		rand2 = cellRadius + (rand() % (SCREEN_HEIGHT - 2 * cellRadius));

		points[i][0] = rand1;
		points[i][1] = rand2;
		points[i][2] = i;

		i++; // next point
	}
	fp.close(); // close file
	cout << "Done writing" << endl;
}
void drawCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat cellRadius, GLint numberOfSides)
{

	int numberOfVertices = numberOfSides + 1;

	GLfloat twicePi = 2.0f * M_PI;

	for (int i = 0; i < numberOfVertices; i++)
	{
		circleVerticesX[i] = x + (cellRadius * cos(i *  twicePi / numberOfSides));
		circleVerticesY[i] = y + (cellRadius * sin(i * twicePi / numberOfSides));
		circleVerticesZ[i] = z;
	}



	for (int i = 0; i < numberOfVertices; i++)
	{
		allCircleVertices[i * 3] = circleVerticesX[i];
		allCircleVertices[(i * 3) + 1] = circleVerticesY[i];
		allCircleVertices[(i * 3) + 2] = circleVerticesZ[i];
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, allCircleVertices);
	glDrawArrays(GL_LINE_STRIP, 0, numberOfVertices);
	glDisableClientState(GL_VERTEX_ARRAY);
}
void print(double points[][3])
{
	for (int i = 0; i < NUM_OF_CELLS; i++)
	{
		cout << "Array is (" << points[i][0] << "," << points[i][1] << ")" << " dim = " << points[i][2] << endl;
	}

}
void printPoint(double points[3], char* p)
{

	cout << p << " is" << "(" << points[0] << "," << points[1] << ")" << " dim = " << points[2] << endl;

}

int main()
{
	Node *root = NULL;
	Node testNode;
	double foundCopy[3];
	//	double *resolved = NULL;
	clock_t begin, end;
	bool collisions_flag = true;

	double tempCollisions = 0;
	double unstableState = 0;
	//Radnom Numbers
	srand(static_cast<unsigned int> (time(NULL)));
	//srand(2);
	//GL
	circleVerticesX = new GLfloat[361];
	circleVerticesY = new GLfloat[361];
	circleVerticesZ = new GLfloat[361];

	GLenum err = glewInit();

	double best_dist;

	double p1[3];
	double p2[3];

	int maxCellsX = SCREEN_WIDTH / (cellRadius * 2);
	int maxCellsY = SCREEN_HEIGHT / (cellRadius * 2);

	int maxCells = maxCellsX*maxCellsY;

	int totalCells = NUM_OF_CELLS;

	/*if (NUM_OF_CELLS > maxCells)
	{
	cout << "Can't Resolve, Number of cells are exceeding box size" << endl << "Max cells possible are " << maxCells << endl;
	return 0;
	}*/

	createRandomPoints();


	//create k-d tree
	for (int i = 0; i < NUM_OF_CELLS; i++)
		root = insert(root, points[i]);



	cout << "Initial Collision Count \n";
	cout << collision_count();
	cout << "End\n";
	// all Check // Working perfect till here///////////////////////////////////////////////
	int collisionCheckTwice = 0;


	float max = 0;
	glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}

	if (!glfwInit())
	{
		return -1;
	}
	// Create a windowed mode window and its OpenGL context
	window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Cells", NULL, NULL);
	allCircleVertices = new GLfloat[(361) * 3];
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	// Make the window's context current
	glfwMakeContextCurrent(window);

	glViewport(0.0f, 0.0f, SCREEN_WIDTH, SCREEN_HEIGHT); // specifies the part of the window to which OpenGL will draw (in pixels), convert from normalised to pixels
	glMatrixMode(GL_PROJECTION); // projection matrix defines the properties of the camera that views the objects in the world coordinate frame. Here you typically set the zoom factor, aspect ratio and the near and far clipping planes
	glLoadIdentity(); // replace the current matrix with the identity matrix and starts us a fresh because matrix transforms such as glOrpho and glRotate cumulate, basically puts us at (0, 0, 0)
	glOrtho(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT, 0, 1); // essentially set coordinate system
	glMatrixMode(GL_MODELVIEW); // (default matrix mode) modelview matrix defines how your objects are transformed (meaning translation, rotation and scaling) in your world
	glLoadIdentity(); // same as above comment

					  // Loop until the user closes the window
					  // end of initialization of opengl
	int runCounter = 0;

	analysis[runCounter][0] = 0;
	analysis[runCounter][1] = collision_count();

	double areaOverlap = 0;
	while (!glfwWindowShouldClose(window))
	{
		glClear(GL_COLOR_BUFFER_BIT);

		while (true)
		{
			runCounter++;
			int collisions = 0;

			glClear(GL_COLOR_BUFFER_BIT);
			begin = clock();
			for (int a = 0; a < NUM_OF_CELLS; a++)
			{
				visited = 0;
				found = 0;
				best_dist = 100000;

				// point under test
				testNode.point[0] = points[a][0];
				testNode.point[1] = points[a][1];
				testNode.point[2] = points[a][2];


				//finding nearest neighbor to testNode
				nearest(root, &testNode, 0, 3, &found, &best_dist);

				//creating a copy of nearest point found
				foundCopy[0] = found->point[0];
				foundCopy[1] = found->point[1];
				foundCopy[2] = found->point[2];

				begin = clock();
				collisions_flag = resolve_collisions(testNode, foundCopy);
				end = clock();
				totalTime += end - begin;
				analysis[runCounter][0] = ((float)(totalTime) / CLOCKS_PER_SEC) * 100;
				analysis[runCounter][1] = collision_count();
				if (collisions_flag)
				{

					int idx1 = resolved[2];
					int idx2 = resolved[5];

					points[idx1][0] = resolved[0];
					points[idx1][1] = resolved[1];
					points[idx1][2] = resolved[2];

					points[idx2][0] = resolved[3];
					points[idx2][1] = resolved[4];
					points[idx2][2] = resolved[5];


					p1[0] = resolved[0];
					p1[1] = resolved[1];
					p1[2] = resolved[2];

					p2[0] = resolved[3];
					p2[1] = resolved[4];
					p2[2] = resolved[5];

					deleteTree(root);
					root = NULL;
					//cout << "after deleting root and inserting back in\n";
					for (int i = 0; i < NUM_OF_CELLS; i++)
						root = insert(root, points[i]);

					collisions += 1;

					//cout << "Resolved at end is " << resolved;
					collisions_flag = false;

				}


			}

			if (collisions == tempCollisions)
			{
				unstableState++;
			}
			else
			{
				tempCollisions = collisions;
				unstableState = 0;
			}

			cout << "Collisions are " << collision_count() << endl;

			if (collision_count() == 0 || unstableState >= 5)
			{

				cout << "Time\n";
				ofstream f;
				f.open("time.txt");

				for (int i = 0; i < runCounter; i++)
				{
					pointVertex[0] = analysis[i][0];
					pointVertex[1] = analysis[i][1];
					f << pointVertex[0] << endl;

				}

				f.close();

				f.open("collisions.txt");
				cout << "collisions\n";
				for (int i = 0; i < runCounter; i++)
				{
					pointVertex[1] = analysis[i][1];

					f << pointVertex[1] << endl;

				}
				f.close();
				cout << "Files written" << endl;
				double time = (float)(totalTime) / CLOCKS_PER_SEC;
				cout << "Total Time to resolve collisions is " << time << endl;

				break;

			}
			if (collisions > 0)
			{
				for (int i = 0; i < NUM_OF_CELLS; i++)
				{
					drawCircle(points[i][0], points[i][1], 0, cellRadius, 360);
				}

				if (NUM_OF_CELLS > maxCells)
				{
					areaOverlap++;
					if (areaOverlap > 5)
					{
						cout << "Cant Converge " << endl;
						cout << "Press any key to continue " << endl;
						break;
					}
				}
			}

			glfwSwapBuffers(window);

			// Poll for and process events
			glfwPollEvents();

		}
		getchar();
		//		cout << "Run Counter is " << runCounter << endl;
		break;
	}

	return 0;
}
