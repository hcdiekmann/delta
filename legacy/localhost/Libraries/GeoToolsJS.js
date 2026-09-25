/**
 * Various geometric classes and utility functions.
 *
 * Copyright (c) 2018 Vincent Peters
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 * @author Vincent Peters (vajpeters@gmail.com)
 *
 * @version 0.1
 *
 * Known issues:
 * Spline display is segmented, true spline interpolation should be added
 */

var BEZIER_CIRCLE = 4*(Math.sqrt(2)-1)/3;
const TWO_THIRD_PI = (Math.PI * 2) / 3;
const ONE_THIRD_PI = Math.PI / 3;

function Ray(point, direction) {
  /**
  * A ray is defined as a point and a direction vector. It represent an endless line thru a point.
  */
  this.point = point;
  this.direction = direction;
};

function Line(start, end) {
  /**
  * A line is defined as section between two points.
  */
  this._start = start;
  this._end = end;
  
  this.__defineSetter__("start", function(start){
    this._start = start;
  });

  this.__defineGetter__("start", function(){
    return this._start;
  });  
 
   this.__defineSetter__("end", function(end){
    this._end = end;
  });

  this.__defineGetter__("end", function(){
    return this._end;
  }); 
 
  /*
  * Returns the vector of the line
   */
  this.direction = function() {
    return glm.sub(this._end, this._start);
  };

  this.length = function() {
    /*
    * Returns the length of the line piece
    */
    return glm.length(glm.sub(this._start,this._end));
  };

  /*
  * Returns the unit vector of the line
   */
  this.unit = function() {
    return glm.normalize(this.direction());
  };


  this.dist = function(geometry) {
    /*
    * Returns the shortest distance between a point or a line piece and a line piece.
    * Can be either the distance to a point on the line or the distance the start or end point.
    *
    * @param point
    */
    if(geometry instanceof glm.vec3){
        var v = vec3.create();
        var w = vec3.create();
        vec3.subtract(v, this._end, this._start);
        vec3.subtract(w, point, this._start);

        var c1 = vec3.dot(v, w);
        if ( c1 <= 0 ) {
          return (new Line(this._start, point)).length();
        };

        var c2 = vec3.dot(v, v);
        if ( c2 <= c1 ) {
          return (new Line(this._end, point)).length();
        };
        var b = c1 / c2;
        var Pb = vec3.create();

        vec3.multiplyScalar(Pb, v, b);
        vec3.add(Pb, Pb, this._start);
        return (new Line(Pb, point)).length();
      };
    if(geometry instanceof Line){
      //Vector   u = S1.P1 - S1.P0;   //this.direction();
      //Vector   v = S2.P1 - S2.P0;   //geometry.direction();
      //Vector   w = S1.P0 - S2.P0;   //glm.sub(this.start, geometry.start);
      
      var a = glm.dot(this.direction(), this.direction()); //float    a = dot(u,u);         // always >= 0
      var b = glm.dot(this.direction(), geometry.direction()); //float    b = dot(u,v);
      var c = glm.dot(geometry.direction(), geometry.direction()); //float    c = dot(v,v);         // always >= 0
      var d = glm.dot(this.direction(), glm.sub(this.start, geometry.start)); //float    d = dot(u,w);
      var e = glm.dot(geometry.direction(), glm.sub(this.start, geometry.start)); //float    e = dot(v,w);
      var D = a * c - b * b; //float    D = a*c - b*b;        // always >= 0
      
      if ((arguments.length > 1) && (arguments[1] == true)){ //if clamping is true
        var sc, sN, sD = D; //float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
        var tc, tN, tD = D; //float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

        // compute the line parameters of the two closest points
        if (D < 0.00000001) { // the lines are almost parallel
          sN = 0.0;         // force using point P0 on segment S1
          sD = 1.0;         // to prevent possible division by 0.0 later
          tN = e;
          tD = c;
        }else {                 // get the closest points on the infinite lines
          sN = (b*e - c*d);
          tN = (a*e - b*d);
          if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
          }else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
          };
        };
        if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
          tN = 0.0;
          // recompute sc for this edge
          if (-d < 0.0){
            sN = 0.0;
          }else if (-d > a){
            sN = sD;
          }else {
            sN = -d;
            sD = a;
          };
        }else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
          tN = tD;
          // recompute sc for this edge
          if ((-d + b) < 0.0){
            sN = 0;
          }else if ((-d + b) > a){
            sN = sD;
          }else {
            sN = (-d +  b);
            sD = a;
          };
        };
        // finally do the division to get sc and tc
        sc = (Math.abs(sN) < 0.00000001 ? 0.0 : sN / sD);
        tc = (Math.abs(tN) < 0.00000001 ? 0.0 : tN / tD);
      }else{ //non clamped situation
        var sc, tc;
        // compute the line parameters of the two closest points
        if (D < 0.00000001) {          // the lines are almost parallel
          sc = 0.0;
          tc = (b>c ? d/b : e/c);    // use the largest denominator
        }else {
          sc = (b*e - c*d) / D;
          tc = (a*e - b*d) / D;
        };
      };
      // get the difference of the two closest points
      var dP = glm.sub(glm.add(glm.sub(this.start, geometry.start),glm.mul(this.direction(), sc)),glm.mul(geometry.direction(), tc));  // =  S1(sc) - S2(tc)
      return glm.length(dP);   // return the closest distance
    };
  };

  /*
  * Returns the angle between the line and a vector
   */
  this.angle = function(vector) {
    return Math.acos(glm.dot(this.unit(), vector));
  };

  /*
  * Returns the position on the line between start and end defined by a number between 0.0 and 1.0
   *
   * @param p
   */
  this.position = function(_p) {
    var p = _p;
    if (p < 0.0) {
      p = 0.0;
    };
    if (p > 1.0) {
      p = 1.0;
    };
    return glm.add(this._start, glm.mul(this.direction(), p));
  };

  this.display = function() {
    line(this._start[0], this._start[1], this._start[2], this._end[0], this._end[1], this._end[2]);
  };
};

function Plane(point, normal) {
  /**
  * A plane is defined as a point and a normal vector.
  */
  this.point = point;
  this.normal = glm.normalize(normal);

  this.project = function(p){
    /**
    * Project p on the plane
    * Uses https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d
    */
    var v = glm.vec3(); 
    v = glm.sub(p, this.point);
    var dist = glm.dot(v, normal);
    var result = glm.vec3();
    result = glm.mul(normal, dist);
    return glm.sub(p, result);
  };

  this.equals = function(plane) {
    /**
    * Determines if the given planes are actually in the same plane.
    *
    * @param plane
    * @return wheter the planes are actually in the same plane.
    */
    return ((glm.dot(this.point, this.normal) == glm.dot(plane.point, plane.normal)) && glm.all(glm.equal(this.normal, plane.normal)));
  };
};

function Circle(center, normal, radius){
  /**
  * A circle is defined as a point, a normal vector and a radius.
  */
  this.center = center;
  this.normal = glm.normalize(normal);
  this.radius = radius;
  
  this.intersect = function(circle) {
    /**
    * Computes the intersection points of two circles.
    *
    * @param circle
    * @return the two intersection points of the circles in line format.
    */

    if (!(glm.dot(circle.center, circle.normal) == glm.dot(this.center, this.normal)) && !glm.all(glm.equal(this.normal, circle.normal))){  //check coplanar
      throw "the two circles are not in the same plane.";
    };
    var d = glm.length(glm.sub(this.center,circle.center));
    var a = (Math.pow(circle.radius,2) - Math.pow(this.radius,2) + Math.pow(d,2))/(2 * d);
    var h = Math.sqrt(Math.pow(circle.radius,2)-Math.pow(a,2));
    
    var v = glm.mul(glm.sub(this.center,circle.center), (1/d));
    var w = glm.cross(circle.normal, v);

    var point1 = glm.add(glm.add(circle.center, glm.mul(v,a)), glm.mul(w,h));
    var point2 = glm.sub(glm.add(circle.center, glm.mul(v,a)), glm.mul(w,h));;
    
    return new Line(point1, point2);
  };
  
  this.tangencyPoints = function(point) {
	/**
    * Computes the tangency points on a circle from a point outside the circle.
    *
    * @param point
    * @return the two intersection points of the circles in line format.
    */
    if (glm.length(glm.sub((new Plane(this.center,this.normal)).project(point),point)) > 0.0001){  //check if in same plane
      throw "the point and the circle are not in the same plane.";
    };
	if (glm.length(glm.sub(this.center,point)) < radius){  //check if the point lays outside the circcle
      throw "the point is not located in outside the circle.";
    };
	var D = Math.sqrt(Math.pow(glm.length(glm.sub(this.center,point)),2) - Math.pow(this.radius,2)); //distance from the point to the tangency points (Pythagoras)
	
	return this.intersect(new Circle(point, this.normal, D));  
  };
  
  this.display = function() {
    //create array of bezier points for a quarter circle
    var bezierPoint = new Array(4); 
    bezierPoint[0] = glm.vec3(0,radius,0);
    bezierPoint[1] = glm.vec3(BEZIER_CIRCLE*radius,radius,0);
    bezierPoint[2] =  glm.vec3(radius,BEZIER_CIRCLE*radius,0);
    bezierPoint[3] =  glm.vec3(radius,0,0);

    var rotationMatrix;
    var i;
    var rotationAxis =  glm.normalize(glm.cross(zAxis(), this.normal));
    var rotationAngle = Math.acos(glm.dot(zAxis(), this.normal));
    
    rotationMatrix = glm.angleAxis(rotationAngle, rotationAxis);
    for(i = 0; i < 4; i++){
      bezierPoint[i] = glm.add((rotationMatrix)['*'](bezierPoint[i]), this.center);
    };
    bezier( bezierPoint[0][0],bezierPoint[0][1],bezierPoint[0][2] , bezierPoint[1][0],bezierPoint[1][1],bezierPoint[1][2] , bezierPoint[2][0],bezierPoint[2][1],bezierPoint[2][2] , bezierPoint[3][0],bezierPoint[3][1],bezierPoint[3][2] );
        
    rotationMatrix = glm.angleAxis(HALF_PI, this.normal);
    for(i = 0; i < 4; i++){
      bezierPoint[i] = glm.add((rotationMatrix)['*'](glm.sub(bezierPoint[i], this.center)), this.center);
    };
    bezier( bezierPoint[0][0],bezierPoint[0][1],bezierPoint[0][2] , bezierPoint[1][0],bezierPoint[1][1],bezierPoint[1][2] , bezierPoint[2][0],bezierPoint[2][1],bezierPoint[2][2] , bezierPoint[3][0],bezierPoint[3][1],bezierPoint[3][2] );
    
    rotationMatrix = glm.angleAxis(HALF_PI, this.normal);
    for(i = 0; i < 4; i++){
      bezierPoint[i] = glm.add((rotationMatrix)['*'](glm.sub(bezierPoint[i], this.center)), this.center);
    };
    bezier( bezierPoint[0][0],bezierPoint[0][1],bezierPoint[0][2] , bezierPoint[1][0],bezierPoint[1][1],bezierPoint[1][2] , bezierPoint[2][0],bezierPoint[2][1],bezierPoint[2][2] , bezierPoint[3][0],bezierPoint[3][1],bezierPoint[3][2] );
    
    rotationMatrix = glm.angleAxis(HALF_PI, this.normal);
    for(i = 0; i < 4; i++){
      bezierPoint[i] = glm.add((rotationMatrix)['*'](glm.sub(bezierPoint[i], this.center)), this.center);
    };
    bezier(bezierPoint[0][0],bezierPoint[0][1],bezierPoint[0][2] , bezierPoint[1][0],bezierPoint[1][1],bezierPoint[1][2] , bezierPoint[2][0],bezierPoint[2][1],bezierPoint[2][2] , bezierPoint[3][0],bezierPoint[3][1],bezierPoint[3][2] );
  };
};

function Sphere(center, radius){
  /**
  * A sphere is defined as a point and a radius.
  */
  this.center = center;
  this.radius = radius;
  
  /**
  * Computes the intersection of a plane on a sphere. Based on the math explained on: 
  *  https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection
  *
  * @param plane
  * @return the circle from the intersection of the plane on the sphere
  */
  this.intersect = function(geometry){
    if(geometry instanceof Plane){
      var rho = glm.dot(geometry.normal, (glm.sub(geometry.point,this.center))); //calculate the distance from the plane to the center of the circle
      if(Math.abs(rho) > this.radius){
        throw "the plane does not intersect the sphere.";
      }    
      var c = glm.add(this.center, glm.mul(geometry.normal, rho));
      var r = Math.sqrt(Math.pow(radius,2) - Math.pow(rho,2));
      return new Circle(c, geometry.normal, r);
    }else if (geometry instanceof Circle){
      return geometry.intersect(this.intersect(new Plane(geometry.center, geometry.normal)));
    }else{
      return null;
    };
  };

  this.display = function(){
    push();
    translate(this.center[0], this.center[1], this.center[2]);
    sphere(this.radius);
    pop();
  }
}

function Spline(n){
  this.point = new Array(n);
  for(var i = 0; i < n; i++){
    this.point[i] = glm.vec3();
  };
  
  this.length = function(){
    var l = 0.0;
    for(var i = 1; i < this.point.length; i++){
      l = l + glm.length(glm.sub(this.point[i], this.point[i-1]));
    };
    return l;
  };
  
  this.dist = function(geometry){
    if(geometry instanceof Plane){
      var l = 0.0;
      for(var i = 1; i < this.point.length; i++){
        var sn = -glm.dot(geometry.normal, glm.sub(this.point[i], geometry.point));
        var sd = glm.dot(geometry.normal, geometry.normal);
        var sb = sn / sd;
        l = l + glm.length(glm.sub(this.point[i], glm.add(this.point[i], glm.mul(geometry.normal, sb))));
      };
    return l;
    };
  };
  
  this.optimizeKallayMEC = function(){
    var n = this.point.length - 1;
    var j = Math.floor((Math.random()*(n-4)) + 2); 
    var i = Math.floor((Math.random() * (n-j-1))+1);
    var V = new Array(this.point.length);
    
    //project U on the xy-plane 
    V[i] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(this.point[i+j],this.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(this.point[i+j],this.point[i]), zAxis()))))['*'](glm.sub(this.point[i-1],this.point[i]));
    V[i+1] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(this.point[i+j],this.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(this.point[i+j],this.point[i]), zAxis()))))['*'](glm.sub(this.point[i],this.point[i+1]));
    V[i+j] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(this.point[i+j],this.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(this.point[i+j],this.point[i]), zAxis()))))['*'](glm.sub(this.point[i+j-1],this.point[i+j]));
    V[i+j+1] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(this.point[i+j],this.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(this.point[i+j],this.point[i]), zAxis()))))['*'](glm.sub(this.point[i+j],this.point[i+j+1]));

    a = (V[i][0] * V[i+1][0] + V[i][1] * V[i+1][1] + V[i+j+1][0] * V[i+j][0] + V[i+j+1][1] * V[i+j][1]);
    b = (V[i][1] * V[i+1][0] - V[i][0] * V[i+1][1] + V[i+j+1][1] * V[i+j][0] - V[i+j+1][0] * V[i+j][1]);
    var rotation = glm.angleAxis(Math.atan2(b,a), glm.normalize(glm.sub(this.point[i+j],this.point[i])));
    
    for(var r = i; r < i+j; r++){
      this.point[r] = glm.add((rotation)['*'](glm.sub(this.point[r],this.point[i])), this.point[i]);
    };
  };
  
  this.exportDSV = function(delimiter){
    //exports the points in a delimiter-separated value format
    var dsvContent = "data:text/csv;charset=utf-8,";
    for(var i = 1; i < this.point.length-1; i++){
      dsvContent += this.point[i][0].toString() + delimiter + this.point[i][1].toString() + delimiter + this.point[i][2].toString() + "\r\n";
    }
    return dsvContent;
  }
  
  this.sigmaKappa = function(){
    var kappa = 0;
    for(var i = 1; i < this.point.length-1; i++){
      
      //kappa = kappa + Math.abs(1 / glm.dot(glm.sub(this.point[i-1], this.point[i]), glm.sub(this.point[i+1], this.point[i])));
      //kappa = kappa + 1 / Math.acos(glm.dot(glm.sub(this.point[i-1], this.point[i]), glm.sub(this.point[i+1], this.point[i])) / (glm.length(glm.sub(this.point[i-1], this.point[i])) * glm.length(glm.sub(this.point[i+1], this.point[i]))));
      kappa = kappa + (Math.acos(glm.dot(glm.sub(this.point[i-1], this.point[i]), glm.sub(this.point[i+1], this.point[i])) / (glm.length(glm.sub(this.point[i-1], this.point[i])) * glm.length(glm.sub(this.point[i+1], this.point[i])))) * glm.length(glm.sub(this.point[i+1], this.point[i])));
    }
    return kappa;
  };
  
  this.inPlane = function(epsilon){
    /**
    * Returns true if there are more than three points in the spline and the sum of the distances of all the points to an average plane is lower than epsilon
    * Note that longer splines are less likely to return true due to the larger amount of points
    */
    if (this.point.length > 2){
      var tPlane = new Plane(this.point[1], glm.cross(glm.sub(this.point[0], this.point[1]), glm.sub(this.point[this.point.length-1], this.point[1])));
      if (this.dist(tPlane) < epsilon){
        return true;
      }else {
        return false;
      }
    }
    return false;
  }
  
  this.display = function(){
    noFill();
    beginShape();
    for(var i = 0; i < this.point.length; i++){
      vertex(this.point[i][0],this.point[i][1],this.point[i][2]);
    };
    endShape();
  };
}

function triliterate(sphere1, sphere2, sphere3){
  /**
  * Computes the trilateration of three spheres.
  *
  * @param sphere1
  * @param sphere2
  * @param sphere3
  * @return the two intersection points of the thee spheres in line format.
  */
  var e_x = glm.normalize(glm.sub(sphere2.center,sphere1.center));
  var i = glm.dot(glm.sub(sphere3.center, sphere1.center), e_x);
  var e_y = glm.normalize(glm.sub(glm.sub(sphere3.center,sphere1.center), glm.mul(e_x,i)));
  var e_z = glm.cross(e_x, e_y);
  var d = glm.length(glm.sub(sphere2.center, sphere1.center));
  var j = glm.dot(e_y, glm.sub(sphere3.center, sphere1.center));
  var x = (sphere1.radius * sphere1.radius - sphere2.radius * sphere2.radius + d * d) / (2*d);
  var y = (sphere1.radius * sphere1.radius - sphere3.radius * sphere3.radius -2*i*x + i*i + j*j) / (2*j);
    
  if(sphere1.radius*sphere1.radius - x*x - y*y < 0){
    throw "the three spheres do not intersect.";
  };
  
  var z = Math.sqrt(sphere1.radius*sphere1.radius - x*x - y*y);
    
  var point1 = glm.add(glm.add(glm.add(sphere1.center, glm.mul(e_x,x)), glm.mul(e_y,y)) ,glm.mul(e_z,z))
  var point2 = glm.sub(glm.add(glm.add(sphere1.center, glm.mul(e_x,x)), glm.mul(e_y,y)) ,glm.mul(e_z,z))
  return new Line(point1, point2);
}

function xAxis(){
  /**
  * Returns a matrix representing the unit vector in x-axis for use in rotation and translation calculations
  *
  * @return matrix with a unit vector in the direction of the x-axis
  */
  return glm.vec3(1, 0, 0);
};

function yAxis(){
  /**
  * Returns a matrix representing the unit vector in y-axis for use in rotation and translation calculations
  *
  * @return matrix with a unit vector in the direction of the y-axis
  */
  return glm.vec3(0, 1, 0);
};

function zAxis(){
  /**
  * Returns a matrix representing the unit vector in z-axis for use in rotation and translation calculations
  *
  * @return matrix with a unit vector in the direction of the z-axis
  */
  return glm.vec3(0, 0, 1);
};

function Point(x, y, z){
  return glm.vec3(x, y, z);
}

function mecKallay(A, UA, B, UB, L, n, steps){
  /**
  * Optimizes the curve to a minimum energy cuve (MEC) according to the algorithm presented by M. Kalley; Method to approximate the space curve of least energy and prescribed length
  * Original M. Kallay algorithm is sensitive to direction of analysis, by making use of random sampling, this is more or less avoided
  *
  * @param steps defines the amount of itterations to be taken
  */
  
  n = n + (n % 2); //'n' must be even, so the value wil be raised to the next even value of 'n'
  //create a spline

  var P = new Spline(n+1);
  P.point[0] = A;
  P.point[1] = glm.add(A, glm.mul(UA, L/n));
  
  P.point[n-1] = glm.sub(B, glm.mul(UB, L/n));
  P.point[n] = B;
  
  //initialize the spline with a sawtooth profile
  var W = glm.mul(glm.cross(UA,UB),Math.sqrt(Math.pow(L-(2*(L/n)),2) - Math.pow(glm.length(glm.sub(P.point[1],P.point[n-1])),2))/(n-2));
  for(var i = 1; i < n; i++){
    if ((i % 2) == 1){
      P.point[i] = glm.add(P.point[1],glm.mul(glm.sub(P.point[n-1],P.point[1]),(i-1)/(n-2)));
    }else{
      P.point[i] = glm.add(glm.add(P.point[1],glm.mul(glm.sub(P.point[n-1],P.point[1]),(i-1)/(n-2))),W);
    }
  }
  
  //optimize the trajectory of the spline to a minimum energy curve
  var V = new Array(n+1);
  var U = new Array(n+1);
  var a;
  var b;
  var j, i; //comment for original M. Kallay
  
  for(var t = 0; t < steps; t++){
  //for(var t = 0; t < Math.ceil(steps / Math.pow(n,2)); t++){
    //for(var j = n-2; j >= 2; j--){ //interval size from large to small
    //for(var j = 2; j <= n-2; j++){ //interval size from small to large, uncomment for original M. Kallay
      //for(var i = n-j-1; i >= 1; i--){ //direction from end to start
      //for(var i = 1; i <= n-j-1; i++){ //direction from start to end, uncomment for original M. Kallay
        //interval size
        j = Math.floor((Math.random()*(n-4)) + 2); //comment for original M. Kallay
        //j = Math.round(Math.random(2, n-2));
        
        //interval position
        i = Math.floor((Math.random() * (n-j-1))+1); //comment for original M. Kallay
                
        //i = Math.round(Math.random(1, n-j-1));
        U[i] = glm.sub(P.point[i-1],P.point[i]);
        U[i+1] = glm.sub(P.point[i],P.point[i+1]);
        U[i+j] = glm.sub(P.point[i+j-1],P.point[i+j]);
        U[i+j+1] = glm.sub(P.point[i+j],P.point[i+j+1]);
  
        //project U on the xy-plane 
        V[i] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(P.point[i+j],P.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(P.point[i+j],P.point[i]), zAxis()))))['*'](U[i]);
        V[i+1] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(P.point[i+j],P.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(P.point[i+j],P.point[i]), zAxis()))))['*'](U[i+1]);
        V[i+j] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(P.point[i+j],P.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(P.point[i+j],P.point[i]), zAxis()))))['*'](U[i+j]);
        V[i+j+1] = (glm.angleAxis(Math.acos(glm.dot(glm.normalize(glm.sub(P.point[i+j],P.point[i])), zAxis())) , glm.normalize(glm.cross(glm.sub(P.point[i+j],P.point[i]), zAxis()))))['*'](U[i+j+1]);

        a = (V[i][0] * V[i+1][0] + V[i][1] * V[i+1][1] + V[i+j+1][0] * V[i+j][0] + V[i+j+1][1] * V[i+j][1]);
        b = (V[i][1] * V[i+1][0] - V[i][0] * V[i+1][1] + V[i+j+1][1] * V[i+j][0] - V[i+j+1][0] * V[i+j][1]);
        var rotation = glm.angleAxis(Math.atan2(b,a), glm.normalize(glm.sub(P.point[i+j],P.point[i])));
        
        for(var r = i; r < i+j; r++){
          P.point[r] = glm.add((rotation)['*'](glm.sub(P.point[r],P.point[i])), P.point[i]);
        };
  };
  return P;
}

math.type.Matrix.prototype.setMatrix = function(i0, i1, j0, j1, X){
  /**
  * Based on JAMA to work conveniently with large matrices
  *
  * Set a submatrix.
  * Parameters:
  * i0 - Initial row index
  * i1 - Final row index
  * j0 - Initial column index
  * j1 - Final column index
  * X - A(i0:i1,j0:j1)
  */
  for (var i = i0; i <= i1; i++){
    for (var j = j0; j <= j1; j++){
      this._data[i][j] = X._data[i-i0][j-j0];
    };
  };
  //write implementation for input X as array
};

math.type.Matrix.prototype.getMatrix = function(i0, i1, j0, j1){
  /**
  * Based on JAMA to work conveniently with large matrices
  *
  * Get a submatrix.
  * Parameters:
  * i0 - Initial row index
  * i1 - Final row index
  * j0 - Initial column index
  * j1 - Final column index
  */
  X = math.zeros(i1 - i0 + 1, j1 - j0 + 1);
  for (var i = i0; i <= i1; i++){
    for (var j = j0; j <= j1; j++){
      X._data[i-i0][j-j0] = this._data[i][j];
    };
  };
  return X;
};

function exportSTL (model){
	//exports p5.Geometry to STL file
    var stlContent = "data:text/stl;charset=utf-8,\r\n";
	stlContent += "solid \r\n";
	if(Array.isArray(model)) {
		for (var h = 0; h < model.length-1; h++){
			for(var i = 0; i < model[h].faces.length-1; i++){
				var facetNormal = ((model[h].vertices[model[h].faces[i][1]].copy().sub(model[h].vertices[model[h].faces[i][0]])).cross(model[h].vertices[model[h].faces[i][2]].copy().sub(model[h].vertices[model[h].faces[i][0]]))).normalize();
				stlContent += "\tfacet normal " + facetNormal["x"] + " " + facetNormal["y"] +  " " + facetNormal["z"] +"\r\n";
				stlContent += "\t\touter loop\r\n";
				stlContent += "\t\t\tvertex " + model[h].vertices[model[h].faces[i][0]]["x"] + " " + model[h].vertices[model[h].faces[i][0]]["y"] +  " " + model[h].vertices[model[h].faces[i][0]]["z"] +"\r\n";
				stlContent += "\t\t\tvertex " + model[h].vertices[model[h].faces[i][1]]["x"] + " " + model[h].vertices[model[h].faces[i][1]]["y"] +  " " + model[h].vertices[model[h].faces[i][1]]["z"] +"\r\n";
				stlContent += "\t\t\tvertex " + model[h].vertices[model[h].faces[i][2]]["x"] + " " + model[h].vertices[model[h].faces[i][2]]["y"] +  " " + model[h].vertices[model[h].faces[i][2]]["z"] +"\r\n";
				stlContent += "\t\tendloop\r\n";
				stlContent += "\tendfacet\r\n";
			}
		}
	}else{
		for(var i = 0; i < model.faces.length-1; i++){
			var facetNormal = ((model.vertices[model.faces[i][1]].copy().sub(model.vertices[model.faces[i][0]])).cross(model.vertices[model.faces[i][2]].copy().sub(model.vertices[model.faces[i][0]]))).normalize();
			stlContent += "\tfacet normal " + facetNormal["x"] + " " + facetNormal["y"] +  " " + facetNormal["z"] +"\r\n";
			stlContent += "\t\touter loop\r\n";
			stlContent += "\t\t\tvertex " + model.vertices[model.faces[i][0]]["x"] + " " + model.vertices[model.faces[i][0]]["y"] +  " " + model.vertices[model.faces[i][0]]["z"] +"\r\n";
			stlContent += "\t\t\tvertex " + model.vertices[model.faces[i][1]]["x"] + " " + model.vertices[model.faces[i][1]]["y"] +  " " + model.vertices[model.faces[i][1]]["z"] +"\r\n";
			stlContent += "\t\t\tvertex " + model.vertices[model.faces[i][2]]["x"] + " " + model.vertices[model.faces[i][2]]["y"] +  " " + model.vertices[model.faces[i][2]]["z"] +"\r\n";
			stlContent += "\t\tendloop\r\n";
			stlContent += "\tendfacet\r\n";
		}
	}
	stlContent += "endsolid \r\n";
    return stlContent;
}

function createArray(length) {
  var arr = new Array(length || 0),
      i = length;

  if (arguments.length > 1) {
    var args = Array.prototype.slice.call(arguments, 1);
    while(i--) arr[i] = createArray.apply(this, args);
  }        
  return arr;
}