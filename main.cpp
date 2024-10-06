#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <GL/glut.h>
#include <vector>
#include <sstream>

#include<string>
#include<tuple>
#include <armadillo>

using namespace std;



void affichage(void);
void clavier(unsigned char touche, int x, int y);
void affiche_repere(void);
/*void afficherCourbe(void);*/


void mouse(int, int, int, int);
void mouseMotion(int, int);
//void reshape(int,int);
float t = .5;

// variables globales pour OpenGL
bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
float mouseX, mouseY;
float cameraAngleX;
float cameraAngleY;
float cameraDistance = 0.;

// constantes pour les materieux
float no_mat[] = { 0.0f, 0.0f, 0.0f, 1.0f };
float mat_ambient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
float mat_ambient_color[] = { 0.8f, 0.8f, 0.2f, 1.0f };
float mat_diffuse[] = { 0.1f, 0.5f, 0.8f, 1.0f };
float mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
float no_shininess = 0.0f;
float low_shininess = 5.0f;
float high_shininess = 100.0f;
float mat_emission[] = { 0.3f, 0.2f, 0.2f, 0.0f };


void initOpenGl()
{

	//lumiere 

	glClearColor(.5, .5, 0.5, 0.0);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat l_pos[] = { 3.,3.5,3.0,1.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, l_pos);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, l_pos);
	glLightfv(GL_LIGHT0, GL_SPECULAR, l_pos);
	glEnable(GL_COLOR_MATERIAL);

	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	//glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	// glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE|GLUT_RGB);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)200 / (GLfloat)200, 0.1f, 10.0f);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0., 0., 4., 0., 0., 0., 0., 1., 0.);

}

int choice = 0;


//control points in each direction
const int numControlPointsU = 4;
const int numControlPointsV = 4;

// Define the degree 
const int degreeU = 2;
const int degreeV = 2;

// knot vectors (nodal) , length = nbr cntrpoints + degree + 1
arma::vec knotVectorU = { 0,0, 1,4,6,8,8 };
arma::vec knotVectorV = { 0,0, 2,4,6,8,8 };

// weights
GLfloat weights[numControlPointsU][numControlPointsV] = {
	{1, 1, 1, 1},
	{1,1, 1, 1},
	{1, 1, 1, 1},
	{1, 1, 1, 1}
};
// Function to calculate the basis function N_i,p(t) using Cox-de Boor recursion
float calculateBasisFunction(int i, int p, float t, const arma::vec& knotVector) {
	if (p == 0) {
		if (knotVector[i] <= t && t < knotVector[i + 1]) {
			return 1.0;
		}

		else {

			return 0.0;
		}
	}

	float left = 0;
	float right = 0;
	float denominator1 = knotVector[i + p] - knotVector[i];
	float denominator2 = knotVector[i + p + 1] - knotVector[i + 1];
    // the 2 next if will serve the 0/0 problem. if its 0 left remain 0, same for right.
	if (denominator1 != 0) {
		left = (t - knotVector[i]) / denominator1;
	}

	if (denominator2 != 0) {
		right = (knotVector[i + p + 1] - t) / denominator2;
	}

	return left * calculateBasisFunction(i, p - 1, t, knotVector) + right * calculateBasisFunction(i + 1, p - 1, t, knotVector);
}



GLfloat ctrlpoints[numControlPointsU][numControlPointsV][3] = {
 {{-1.5,-0.5, -1.5},  {-0.5, -0.5,-1.5 }, {0.5, -0.5, -1.5 }, {1.5, -0.5,-1.5}, },
 {{-1.5, -0.5, -0.5},  {-0.5, 1.0,-0.5 }, {0.5, 1.0, -0.5 }, {1.5, 1,-0.5},  },
 {{-1.5, 1,  0.5},  {-0.5, 1, 0.5 }, {0.5, 1.0,  0.5 }, {1.5, -0.5, 0.5}, },
 {{-1.5, -0.5,  1.5},  {-0.5, -0.5, 1.5 }, {0.5, -0.5,  1.5 }, {1.5, -0.5, 1.5}, }


};

// Function to evaluate the NURBS surface at parameters u and v
arma::vec evaluateNURBSSurface(float u, float v) {
	arma::vec result(3, arma::fill::zeros);

	for (int i = 0; i < numControlPointsU; ++i) {
		float basisU = calculateBasisFunction(i, degreeU, u, knotVectorU);
		for (int j = 0; j < numControlPointsV; ++j) {
			float basisV = calculateBasisFunction(j, degreeV, v, knotVectorV);
			for (int k = 0; k < 3; ++k) {
				result(k) += ctrlpoints[i][j][k] * weights[i][j] * basisU * basisV;
			}

		}

	}
	// dividing by the sum of basis functions times weights
	float sumBasisWeights = 0.0;
	for (int i = 0; i < numControlPointsU; ++i) {
		float basisU = calculateBasisFunction(i, degreeU, u, knotVectorU);
		for (int j = 0; j < numControlPointsV; ++j) {
			float basisV = calculateBasisFunction(j, degreeV, v, knotVectorV);
			sumBasisWeights += weights[i][j] * basisU * basisV;
		}
	}

	
	// Check to avoid division by zero
	if (sumBasisWeights != 0.0) {
		result /= sumBasisWeights;
		
	}
	else {
		
		
	}

	return result;



}


//  1st derivative in direction u
arma::vec computeFirstDerivativeFiniteDifferenceU(float u, float v) {
	const float epsilon = 1e-5;

	// Perturb the parameter in the u-direction
	float uPlusEpsilon = u + epsilon;
	float uMinusEpsilon = u - epsilon;

	// Calculate the NURBS surface values at perturbed u values
	arma::vec surfaceUPlusEpsilon = evaluateNURBSSurface(uPlusEpsilon, v);
	arma::vec surfaceUMinusEpsilon = evaluateNURBSSurface(uMinusEpsilon, v);

	// Calculate the finite difference approximation for the u-direction
	arma::vec derivativeU = (surfaceUPlusEpsilon - surfaceUMinusEpsilon) / (2 * epsilon);
	derivativeU /= arma::norm(derivativeU);

	return derivativeU;
}

//  2nd derivative in direction u
arma::vec computeSecondeDerivativeFiniteDifferenceU(float u, float v) {
	const float epsilon = 1e-5;

	// Perturb the parameter in the u-direction
	float uPlusEpsilon = u + epsilon;
	float uMinusEpsilon = u - epsilon;

	// Calculate the first derivatives at perturbed u values
	arma::vec derivative_uPlusEpsilon = computeFirstDerivativeFiniteDifferenceU(uPlusEpsilon, v);
	arma::vec derivative_uMinusEpsilon = computeFirstDerivativeFiniteDifferenceU(uMinusEpsilon, v);

	// Calculate the second derivative approximation for the u-direction
	arma::vec second_derivative = (derivative_uPlusEpsilon - derivative_uMinusEpsilon) / (2 * epsilon);

	return second_derivative;
}

//  1st derivative in direction v
arma::vec computeFirstDerivativeFiniteDifferenceV(float u, float v) {
	const float epsilon = 1e-5;

	// Perturb the parameter in the v-direction
	float vPlusEpsilon = v + epsilon;
	float vMinusEpsilon = v - epsilon;

	// Calculate the NURBS surface values at perturbed v values
	arma::vec surfaceVPlusEpsilon = evaluateNURBSSurface(u, vPlusEpsilon);
	arma::vec surfaceVMinusEpsilon = evaluateNURBSSurface(u, vMinusEpsilon);

	// Calculate the finite difference approximation for the v-direction
	arma::vec derivativeV = (surfaceVPlusEpsilon - surfaceVMinusEpsilon) / (2 * epsilon);
	derivativeV /= arma::norm(derivativeV);
	//cerr << "r:  " << arma::norm(derivativeV) << endl;
	return derivativeV;
}
//  2nd derivative in direction v
arma::vec computeSecondeDerivativeFiniteDifferenceV(float u, float v) {
	const float epsilon = 1e-5;

	// Perturb the parameter in the v-direction
	float vPlusEpsilon = v + epsilon;
	float vMinusEpsilon = v - epsilon;

	// Calculate the first derivatives at perturbed v values
	arma::vec derivative_vPlusEpsilon = computeFirstDerivativeFiniteDifferenceV(u, vPlusEpsilon);
	arma::vec derivative_vMinusEpsilon = computeFirstDerivativeFiniteDifferenceV(u, vMinusEpsilon);

	// Calculate the second derivative approximation for the v-direction
	arma::vec second_derivative = (derivative_vPlusEpsilon - derivative_vMinusEpsilon) / (2 * epsilon);

	return second_derivative;
}

// mixed derivative dudv (the cross-derivative term between the parameters uu and vv)
arma::vec computeMixedDerivativeFiniteDifference(float u, float v) {
	const float epsilonu = 1e-5;
	const float epsilonv = 1e-5;

	// Calculate the first derivatives at perturbed u and v values
	arma::vec one = evaluateNURBSSurface(u + epsilonu, v + epsilonv);
	arma::vec two = evaluateNURBSSurface(u + epsilonu, v - epsilonv);
	arma::vec three = evaluateNURBSSurface(u - epsilonu, v + epsilonv);
	arma::vec four = evaluateNURBSSurface(u - epsilonu, v - epsilonv);

	// Calculate the mixed derivative approximation
	arma::vec mixed_derivative = (one - two - three + four) / (4 * epsilonu * epsilonv);

	return mixed_derivative;
}

// Function to compute the tangent vectors and normal vector at parameters u and v
void computeSurfaceFrame(float u, float v, arma::vec3& tangentU, arma::vec3& tangentV, arma::vec3& normal) {

	tangentU.zeros();
	tangentV.zeros();
	normal.zeros();


	// Compute the tangent vectors in both u and v directions
	tangentU = computeFirstDerivativeFiniteDifferenceU(u, v);
	tangentU /= arma::norm(tangentU);
	tangentV = computeFirstDerivativeFiniteDifferenceV(u, v);
	tangentV /= arma::norm(tangentV);

	// Compute the normal vector
	normal = arma::cross(tangentU, tangentV);
	normal /= arma::norm(normal);




}
// gaussian curvature using first & second fundementals
float computeCurvature(float u, float v) {
	// Compute the first derivatives with respect to u and v
	arma::vec du = computeFirstDerivativeFiniteDifferenceU(u, v);
	arma::vec dv = computeFirstDerivativeFiniteDifferenceV(u, v);


	// Compute the second derivatives with respect to u and v
	arma::vec duu = computeSecondeDerivativeFiniteDifferenceU(u, v);
	arma::vec dvv = computeSecondeDerivativeFiniteDifferenceV(u, v);
	arma::vec duv = computeMixedDerivativeFiniteDifference(u, v);

	// Calculate the coefficients of the first fundamental form
	float E = arma::dot(du, du);
	float F = arma::dot(du, dv);
	float G = arma::dot(dv, dv);



	// Calculate the coefficients of the second fundamental form
	arma::vec normal = arma::cross(du, dv);
	normal /= arma::norm(normal);

	float L = arma::dot(duu, normal);
	float M = arma::dot(duv, normal);
	float N = arma::dot(dvv, normal);



	float K = (L * N - (M * M)) / (E * G - (F * F));

	// also we can Calculate the mean curvature 
	float H = (E * N - 2 * F * M + G * L) / (2 * (E * G - F * F));

	
	
	return H; // K for gaussian or H for mean
}

arma::vec mapCurvatureToColor(float curvature) {
	float r = 0;
	float g = 0;
	float b = 0;
	if (curvature < 0) {
		r = 0.0; g = 0.0; b = 1.0; // Blue for negative curvature
	}


	else if (curvature == 0) { 
		r = 0.0; g = 0.0; b = 0.0; //black for curvature near to zero
	}
	else if (curvature > 0) { 
		r = 1.0; g = 0.0; b = 0.0; // red for positive curvature
	}



	arma::vec col = { r, g, b };
	return col;
}

// just to link control points as polygone
void drawcontrolPolygone(){
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
    for (int i = 0; i < numControlPointsU; ++i) {
        for (int j = 0; j < numControlPointsV; ++j) {
            
            if (j < numControlPointsV - 1) {
                glVertex3fv(ctrlpoints[i][j]);
                glVertex3fv(ctrlpoints[i][j + 1]);
            }
            
            if (i < numControlPointsU - 1) {
                glVertex3fv(ctrlpoints[i][j]);
                glVertex3fv(ctrlpoints[i + 1][j]);
            }
        }
    }
    glEnd();
	
}
// Function to draw the NURBS surface
void drawNURBSSurface(bool curvature) {
     
	glBegin(GL_QUADS);
	for (float u = knotVectorU(degreeU); u <= knotVectorU(numControlPointsU); u += 0.1) {
		for (float v = knotVectorV(degreeV); v <= knotVectorV(numControlPointsV); v += 0.1) {

            if(curvature==true){ //case using curvature

			float curvature = computeCurvature(u, v);
			arma::vec curvaturecolor = mapCurvatureToColor(curvature);
			glColor3f(curvaturecolor(0), curvaturecolor(1), curvaturecolor(2));

			//cout << "for u,v :  " << u << "," << v <<endl;
            //cout << "curvature value:  " << curvature << endl;
			


			}
			else{ // without curvature just give any color 

				glColor3f(0.1,0.6,0.5);
			}
            
			// drawing my surface using quads 
			arma::vec p = evaluateNURBSSurface(u, v);
			arma::vec p1 = evaluateNURBSSurface(u, v + 0.1);
			arma::vec p2 = evaluateNURBSSurface(u + 0.1, v + 0.1);
			arma::vec p3 = evaluateNURBSSurface(u + 0.1, v);
			glVertex3f(p(0), p(1), p(2));
			glVertex3f(p1(0), p1(1), p1(2));
			glVertex3f(p2(0), p2(1), p2(2));
			glVertex3f(p3(0), p3(1), p3(2));

		}

	}
	glEnd();

	

}
// initialization in directions u & v  just for the tangentU,tangentV, Normal to be in the center of the surface, then u can move using u,U,v,V keys...
float uu = ( knotVectorU(degreeU)+ knotVectorU(numControlPointsU) )/2;
float vv = (knotVectorV(degreeV)+knotVectorV(numControlPointsV))/2;

void draw_tangentU_tangentV_normal(){

        arma::vec S = evaluateNURBSSurface(uu, vv);
		arma::vec3 tangentU, tangentV, normal;
		computeSurfaceFrame(uu, vv, tangentU, tangentV, normal);
		float scalingFactor = 0.5; 

		// Draw the tangentU vector
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_LINES);
		glVertex3f(S(0), S(1), S(2));
		glVertex3f(S(0) + scalingFactor * tangentU(0), S(1) + scalingFactor * tangentU(1), S(2) + scalingFactor * tangentU(2));
		glEnd();

		// Draw the tangentV vector
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(S(0), S(1), S(2));
		glVertex3f(S(0) + scalingFactor * tangentV(0), S(1) + scalingFactor * tangentV(1), S(2) + scalingFactor * tangentV(2));
		glEnd();

		// Draw the normal vector
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
		glVertex3f(S(0), S(1), S(2));
		glVertex3f(S(0) + scalingFactor * normal(0), S(1) + scalingFactor * normal(1), S(2) + scalingFactor * normal(2));
		glEnd();

}


void afficherCourbe()
{

	// to draw cntrl points
	glColor3f(1.0, 1.0, 1.0);
    glPointSize(5.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < numControlPointsU; ++i) {
		for (int j = 0; j < numControlPointsV; ++j) {
			glVertex3fv(ctrlpoints[i][j]);
		}
	}
	glEnd();


	if (choice == 0) {  //surface with one color.


          drawcontrolPolygone();
		  drawNURBSSurface(false);
		

	}
    
    	if (choice == 1) { //  surface+ tangentU+tangentV+Normal vecotors 

          drawcontrolPolygone();
		  drawNURBSSurface(false);
		  draw_tangentU_tangentV_normal();

		

	}

		if (choice == 2) { // for the curvature 
          

		  drawcontrolPolygone();
		  drawNURBSSurface(true);

		

	}





}






int main(int argc, char** argv)
{





	/* initialisation de glut et creation
	       de la fenetre */
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB);
	glutInitWindowPosition(200, 200);
	glutInitWindowSize(400, 400);
	glutCreateWindow("ifs");
	/* Initialisation d'OpenGL */
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glColor3f(1.0, 1.0, 1.0);
	glPointSize(1.0);

	//ifs = new Ifs();
/* enregistrement des fonctions de rappel */
	glutDisplayFunc(affichage);


	glutKeyboardFunc(clavier);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	//-------------------------------


	//-------------------------------
	initOpenGl();
	//-------------------------------

/* Entree dans la boucle principale glut */
	glutMainLoop();

	return 0;
}
//------------------------------------------------------
void affiche_repere(void)
{  /*
	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);
	glVertex2f(0., 0.);
	glVertex2f(1., 0.);
	glEnd();

	glBegin(GL_LINES);
	glColor3f(0.0, 1.0, 0.0);
	glVertex2f(0., 0.);
	glVertex2f(0., 1.);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0., 0., 0.);
	glVertex3f(0., 0., 1.);
	glEnd();
	*/
}

//-----------------------------------------------------



//------------------------------------------------------
void affichage(void)
{
	glMatrixMode(GL_MODELVIEW);
	/* effacement de l'image avec la couleur de fond */
	  //	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//       glClearDepth(10.0f);                         // 0 is near, >0 is far

	glPushMatrix();
	glTranslatef(0, 0, cameraDistance);
	glRotatef(cameraAngleX, 1., 0., 0.);
	glRotatef(cameraAngleY, 0., 1., 0.);
	affiche_repere();
	afficherCourbe();

	glPopMatrix();

	/* on force l'affichage du resultat */

	glFlush();
	glutSwapBuffers();

}

//------------------------------------------------------


//------------------------------------------------------
void clavier(unsigned char touche, int x, int y)
{

	switch (touche)
	{

	case 'a': /* surface*/
		choice = 0;
		glutPostRedisplay();
		break;
	case 'z': /*surface + tangentU+tangentV + normal*/
		choice = 1;
		glutPostRedisplay();
		break;
	case 'e': /* curvature*/
		choice = 2;
		glutPostRedisplay();
		break;
	case 'r': /* frenet & cercle osculateur*/
		choice =3;
		glutPostRedisplay();
		break;

	case 'u': // mouvement de repére sur U
		uu += 0.1;
		if (uu > knotVectorU(numControlPointsU)+0.1) uu = knotVectorU(numControlPointsU)+0.1;
		glutPostRedisplay();
		break;
	case 'U': // mouvement de repére sur U
		uu -= 0.1;
		if (uu < knotVectorU(degreeU) ) uu = knotVectorU(degreeU) ;
		glutPostRedisplay();
		break;

	case 'v': // mouvement de repére sur V
		vv += 0.1;
		if (vv > knotVectorV(numControlPointsV) + 0.1) vv = knotVectorV(numControlPointsV) + 0.1;
		glutPostRedisplay();
		break;
	case 'V': // mouvement de repére sur V
		vv -= 0.1;
		if (vv < knotVectorV(degreeV)) vv = knotVectorV(degreeV);
		glutPostRedisplay();
		break;
	

	case 'f': //* affichage en mode fil de fer 
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glutPostRedisplay();
		break;
	case 'p': //* affichage du carre plein 
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glutPostRedisplay();
		break;
	case 's': //* Affichage en mode Points seuls 
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		glutPostRedisplay();
		break;

	case 'q': //*la touche 'q' permet de quitter le programme 
		exit(0);
}
}
void mouse(int button, int state, int x, int y)
{
	mouseX = x;
	mouseY = y;

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseLeftDown = true;
		}
		else if (state == GLUT_UP)
			mouseLeftDown = false;
	}

	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
		}
		else if (state == GLUT_UP)
			mouseRightDown = false;
	}

	else if (button == GLUT_MIDDLE_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseMiddleDown = true;
		}
		else if (state == GLUT_UP)
			mouseMiddleDown = false;
	}
}


void mouseMotion(int x, int y)
{
	if (mouseLeftDown)
	{
		cameraAngleY += (x - mouseX);
		cameraAngleX += (y - mouseY);
		mouseX = x;
		mouseY = y;
	}
	if (mouseRightDown)
	{
		cameraDistance += (y - mouseY) * 0.2f;
		mouseY = y;
	}

	glutPostRedisplay();
}


