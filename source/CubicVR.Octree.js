CubicVR.RegisterModule("Octree",function (base) {

  var undef = base.undef;
  var GLCore = base.GLCore;
  var Plane = CubicVR.plane;
  var Sphere = CubicVR.sphere;
  var enums = CubicVR.enums;
  var dot = CubicVR.vec3.dot;
  var planeMath = CubicVR.vec3;

  enums.frustum = {
    plane: {
      LEFT: 0,
      RIGHT: 1,
      TOP: 2,
      BOTTOM: 3,
      NEAR: 4,
      FAR: 5
    }
  };
    
  enums.octree = {
      T_NW: 0,
      T_NE: 1,
      T_SE: 2,
      T_SW: 3,
      B_NW: 4,
      B_NE: 5,
      B_SE: 6,
      B_SW: 7
    };
  

  var bases = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];

  var sphereMath = {
    intersectsSphere: function ( sphere1, sphere2 ) {
          diff = [ sphere2[0] - sphere1[0], sphere2[1] - sphere1[1], sphere2[2] - sphere1[2] ],
          mag = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2],
          sqrtRad = sphere2[3] + sphere1[3];
          // no need to sqrt here
      return mag <= sqrtRad*sqrtRad;
    },
    intersectsAABB: function ( sphere, aabb ) {
      var min = aabb[0],
          max = aabb[1];
      max = [ max[0] - dims[0], max[1] - dims[1], max[2] - dims[2] ];
      min = [ min[0] - dims[0], min[1] - dims[1], min[2] - dims[2] ];
      max = max[0]*max[0] + max[1]*max[1] + max[2]*max[2];
      min = min[0]*min[0] + min[1]*min[1] + min[2]*min[2];
      var sqr = sphere[3]*sphere[3];
      return max > sqr && min > sqr;
    }
  };

  var Node = function( options ) {
    options = options || {};

    var leaves = [],
        that = this,
        aabb,
        dirty = true;

    this.type = options.type;
    this.object = options.object;
    this.rootTree = undefined;
    

    
    this.posSet = function( pos ) //This is a shortcut for now
    {
        position = pos;
    };

    this.inserted = function( root ) {
      if ( options.inserted ) {
        options.inserted( root );
      } //if
    }; //inserted

    Object.defineProperty( this, "aabb", {
      configurable: true,
      get: function() {
            var mat4 = CubicVR.mat4;
            var vec3 = CubicVR.vec3;
            if (dirty) {
                var p = new Array(8);

                that.object.doTransform();

                var aabbMin;
                var aabbMax;

                if (that.object.obj) {
                    if (!that.object.obj.bb) {
                        aabb = [vec3.add([-1, -1, -1], that.object.position), vec3.add([1, 1, 1], that.object.position)];
                        return aabb;
                    }

                    aabbMin = that.object.obj.bb[0];
                    aabbMax = that.object.obj.bb[1];
                }

                if (!that.object.obj || aabbMin === undef || aabbMax === undef) {
                    aabb = [vec3.add([-1, -1, -1], that.object.position), vec3.add([1, 1, 1], that.object.position)];
                    return aabb;
                }

                var obj_aabb = aabbMin;
                var obj_bounds = vec3.subtract(aabbMax, aabbMin);

                p[0] = [obj_aabb[0], obj_aabb[1], obj_aabb[2]];
                p[1] = [obj_aabb[0], obj_aabb[1], obj_aabb[2] + obj_bounds[2]];
                p[2] = [obj_aabb[0] + obj_bounds[0], obj_aabb[1], obj_aabb[2]];
                p[3] = [obj_aabb[0] + obj_bounds[0], obj_aabb[1], obj_aabb[2] + obj_bounds[2]];
                p[4] = [obj_aabb[0], obj_aabb[1] + obj_bounds[1], obj_aabb[2]];
                p[5] = [obj_aabb[0], obj_aabb[1] + obj_bounds[1], obj_aabb[2] + obj_bounds[2]];
                p[6] = [obj_aabb[0] + obj_bounds[0], obj_aabb[1] + obj_bounds[1], obj_aabb[2]];
                p[7] = [obj_aabb[0] + obj_bounds[0], obj_aabb[1] + obj_bounds[1], obj_aabb[2] + obj_bounds[2]];

                var aabbTest;

                aabbTest = mat4.vec3_multiply(p[0], that.object.tMatrix);

                aabbMin = [aabbTest[0], aabbTest[1], aabbTest[2]];
                aabbMax = [aabbTest[0], aabbTest[1], aabbTest[2]];

                for (var i = 1; i < 8; ++i) {
                    aabbTest = mat4.vec3_multiply(p[i], that.object.tMatrix);

                    if (aabbMin[0] > aabbTest[0]) {
                        aabbMin[0] = aabbTest[0];
                    }
                    if (aabbMin[1] > aabbTest[1]) {
                        aabbMin[1] = aabbTest[1];
                    }
                    if (aabbMin[2] > aabbTest[2]) {
                        aabbMin[2] = aabbTest[2];
                    }

                    if (aabbMax[0] < aabbTest[0]) {
                        aabbMax[0] = aabbTest[0];
                    }
                    if (aabbMax[1] < aabbTest[1]) {
                        aabbMax[1] = aabbTest[1];
                    }
                    if (aabbMax[2] < aabbTest[2]) {
                        aabbMax[2] = aabbTest[2];
                    }
                }

                aabb[0] = aabbMin;
                aabb[1] = aabbMax;

                dirty = false;
            }

            return aabb;
      },
      set: function( val ) {
        dirty = true;
        aabb = val;
        /*position = [
          aabb[ 0 ][ 0 ] + ( aabb[ 1 ][ 0 ] - aabb[ 0 ][ 0 ] ) / 2,
          aabb[ 0 ][ 1 ] + ( aabb[ 1 ][ 0 ] - aabb[ 0 ][ 1 ] ) / 2,
          aabb[ 0 ][ 2 ] + ( aabb[ 1 ][ 0 ] - aabb[ 0 ][ 2 ] ) / 2
        ];*///can't be used for now
      }
    });

    that.aabb = options.aabb || [ [ 0, 0, 0 ], [ 0, 0, 0 ] ];

    var octreeAABB = that.object.position.slice();

    this.addLeaf = function( tree ) {
      var idx = leaves.indexOf( tree );
      if ( idx === -1 ) {
        leaves.push( tree );
        var treeAABB = tree.aabb;
        CubicVR.aabb.engulf( octreeAABB, treeAABB[0] );
        CubicVR.aabb.engulf( octreeAABB, treeAABB[1] );
      } //if
    }; //addLead

    this.removeLeaf = function( tree ) {
      var idx = leaves.indexOf( tree );
      if ( idx > -1 ) {
        leaves.splice( idx, 1 );
      } //if
    }; //addLeaf

    this.destroy = function () {
      leaves = [];
      that.rootTree = undefined;
    }; //destroy

    this.adjust = function() {
      if ( !dirty ) return;
      if (this.rootTree == undefined) return;

      var aabb = this.aabb,
          taabb = this.rootTree.aabb,
          pMin = aabb[ 0 ], pMax = aabb[ 1 ],
          tMin = taabb[ 0 ], tMax = taabb[ 1 ];

      if (  leaves.length > 0 &&
            ( pMin[ 0 ] < tMin[ 0 ] || pMin[ 1 ] < tMin[ 1 ] || pMin[ 2 ] < tMin[ 2 ] ||
              pMax[ 0 ] > tMax[ 0 ] || pMax[ 1 ] > tMax[ 1 ] || pMax[ 2 ] > tMax[ 2 ] ) ) {

        for ( var i=0, l=leaves.length; i<l; ++i ) {
          leaves[i].remove( that );
        } //for

        leaves = [];

        var oldRootTree = that.rootTree;
        that.rootTree = undefined;

        if ( oldRootTree ) {

          while ( true ) {
            var oldRootAABB = oldRootTree.aabb;
            if ( !CubicVR.aabb.containsPoint( aabb[ 0 ], oldRootAABB ) ||
                 !CubicVR.aabb.containsPoint( aabb[ 1 ], oldRootAABB ) ) {
              if ( oldRootTree.root !== undefined ) {
                oldRootTree = oldRootTree.root;
              }
              else {
                break;
              } //if
            }
            else {
              break;
            } //if
          } //while
          CubicVR.aabb.reset( octreeAABB, that.object.position );
          oldRootTree.insert( that );
        } //if
      } //if

    }; //adjust
    
    //should have this global but w/e
    //this.reset = function() {CubicVR.aabb.reset(this.aabb, this.position);};

    return this;
  }; //Node

  var Octree = function( options ) {

    var Tree = function( options ) {
      options = options || {};
      var dirty = false,
          children = [],
          nodes = [],
          depth = options.depth || 0,
          size = options.size || 0,
          hSize = size/2,
          position = options.position || [ 0, 0, 0 ],
          sphere = position.slice().concat( Math.sqrt( 3 * size / 2 * size / 2 ) ),
          aabb = [
            [ position[ 0 ] - hSize, position[ 1 ] - hSize, position[ 2 ] - hSize ],
            [ position[ 0 ] + hSize, position[ 1 ] + hSize, position[ 2 ] + hSize ],
          ],
          root = options.root,

          T_NW = 0,
          T_NE = 1,
          T_SE = 2,
          T_SW = 3,
          B_NW = 4,
          B_NE = 5,
          B_SE = 6,
          B_SW = 7

          that = this;

      Object.defineProperty( this, "position", {
        get: function() { return position; }
      });
      Object.defineProperty( this, "aabb", {
        get: function() { return aabb; }
      });
      Object.defineProperty( this, "numChildren", {
        get: function() {
          var num = 0;
          for ( var i=0; i<8; ++i ) {
            num += children[ i ] ? 1 : 0;
          } //for
          return num;
        }
      });
      Object.defineProperty( this, "children", {
        get: function() { return children; }
      });
      Object.defineProperty( this, "size", {
        get: function() { return size; }
      });
      Object.defineProperty( this, "nodes", {
        get: function() { return nodes; }
      });
      Object.defineProperty( this, "root", {
        get: function() { return root; }
      });

      function $insertNode( node, root ) {
        nodes.push( node );
        node.addLeaf( that );
        node.rootTree = root;
        node.inserted( root );
      }; //$insertNode

      this.remove = function( node ) {
        var idx = nodes.indexOf( node );
        if ( idx > -1 ) {
          nodes.splice( idx, 1 );
        }
      }; //remove

      this.insert = function( node ) {
        if ( depth === 0 ) {
          $insertNode( node, that );
          return;
        } //if

        var p = position,
            aabb = node.aabb,
            min = aabb[ 0 ],
            max = aabb[ 1 ],
            tNW = min[ 0 ] < p[ 0 ] && min[ 1 ] < p[ 1 ] && min[ 2 ] < p[ 2 ],
            tNE = max[ 0 ] > p[ 0 ] && min[ 1 ] < p[ 1 ] && min[ 2 ] < p[ 2 ],
            bNW = min[ 0 ] < p[ 0 ] && max[ 1 ] > p[ 1 ] && min[ 2 ] < p[ 2 ],
            bNE = max[ 0 ] > p[ 0 ] && max[ 1 ] > p[ 1 ] && min[ 2 ] < p[ 2 ],
            tSW = min[ 0 ] < p[ 0 ] && min[ 1 ] < p[ 1 ] && max[ 2 ] > p[ 2 ],
            tSE = max[ 0 ] > p[ 0 ] && min[ 1 ] < p[ 1 ] && max[ 2 ] > p[ 2 ],
            bSW = min[ 0 ] < p[ 0 ] && max[ 1 ] > p[ 1 ] && max[ 2 ] > p[ 2 ],
            bSE = max[ 0 ] > p[ 0 ] && max[ 1 ] > p[ 1 ] && max[ 2 ] > p[ 2 ],
            numInserted = 0;

        if ( tNW && tNE && bNW && bNE && tSW && tSE && bSW && bSE ) {
          $insertNode( node, that );
        }
        else {
          var newSize = size/2,
              offset = size/4,
              x = p[ 0 ], y = p[ 1 ], z = p[ 2 ];

          var news = [
            [ tNW, T_NW, [ x - offset, y - offset, z - offset ] ],
            [ tNE, T_NE, [ x + offset, y - offset, z - offset ] ],
            [ bNW, B_NW, [ x - offset, y + offset, z - offset ] ],
            [ bNE, B_NE, [ x + offset, y + offset, z - offset ] ],
            [ tSW, T_SW, [ x - offset, y - offset, z + offset ] ],
            [ tSE, T_SE, [ x + offset, y - offset, z + offset ] ],
            [ bSW, B_SW, [ x - offset, y + offset, z + offset ] ],
            [ bSE, B_SE, [ x + offset, y + offset, z + offset ] ]
          ];

          for ( var i=0; i<8; ++i ) {
            if ( news[ i ][ 0 ] ) {
              if ( !children[ news[ i ][ 1 ] ] ) {
                children[ news[ i ][ 1 ] ] = new Tree({
                  size: newSize,
                  depth: depth - 1,
                  root: that,
                  position: news[ i ][ 2 ]
                });
              }
              children[ news[ i ][ 1 ] ].insert( node );
              ++numInserted;
            } //if
          }

          if ( numInserted > 1 || !node.rootTree ) {
            node.rootTree = that;
          }

        } //if

      }; //insert

      this.clean = function() {
        var importantChildren = 0;
        for ( var i=0; i<8; ++i ) {
          if ( children [ i ] ) {
            var isClean = children[ i ].clean();
            if ( isClean ) {
              children[ i ] = undefined;
            }
            else {
              ++importantChildren;
            }
          }
        } //for
        if ( nodes.length === 0 && importantChildren === 0 ) {
          return true;
        } //if
        return false;
      }; //clean

    }; //Tree

    options = options || {};
    var size = options.size || 0,
        depth = options.depth || 0;

    if ( size <= 0 ) {
      throw new Error( "Octree needs a size > 0" );
    } //if

    if ( depth <= 0 ) {
      throw new Error( "Octree needs a depth > 0" );
    } //if

    var root = new Tree({
      size: size,
      depth: depth
    });

    this.insert = function( node ) {
      root.insert( node );
    }; //insert

    this.clean = function() {
      root.clean();
    }; //clean

    Object.defineProperty( this, "root", {
      get: function() { return root; }
    });

    Object.defineProperty( this, "size", {
      get: function() { return size; }
    });

  }; //Octree

  Octree.Node = Node;
  //Octree.enums = enums

var Frustum = function() {
  this.last_in = [];
  this._planes = [];
  this.sphere = null;
  for (var i = 0; i < 6; ++i) {
    this._planes[i] = [0, 0, 0, 0];
  } //for
}; //Frustum::Constructor
Frustum.prototype.extract = function(camera, mvMatrix, pMatrix) {
  var mat4 = CubicVR.mat4,
      vec3 = CubicVR.vec3;

  if (mvMatrix === undef || pMatrix === undef) {
    return;
  }
  var comboMatrix = mat4.multiply(pMatrix, mvMatrix);

  var planes = this._planes;
  // Left clipping plane
  planes[enums.frustum.plane.LEFT][0] = comboMatrix[3] + comboMatrix[0];
  planes[enums.frustum.plane.LEFT][1] = comboMatrix[7] + comboMatrix[4];
  planes[enums.frustum.plane.LEFT][2] = comboMatrix[11] + comboMatrix[8];
  planes[enums.frustum.plane.LEFT][3] = comboMatrix[15] + comboMatrix[12];

  // Right clipping plane
  planes[enums.frustum.plane.RIGHT][0] = comboMatrix[3] - comboMatrix[0];
  planes[enums.frustum.plane.RIGHT][1] = comboMatrix[7] - comboMatrix[4];
  planes[enums.frustum.plane.RIGHT][2] = comboMatrix[11] - comboMatrix[8];
  planes[enums.frustum.plane.RIGHT][3] = comboMatrix[15] - comboMatrix[12];

  // Top clipping plane
  planes[enums.frustum.plane.TOP][0] = comboMatrix[3] - comboMatrix[1];
  planes[enums.frustum.plane.TOP][1] = comboMatrix[7] - comboMatrix[5];
  planes[enums.frustum.plane.TOP][2] = comboMatrix[11] - comboMatrix[9];
  planes[enums.frustum.plane.TOP][3] = comboMatrix[15] - comboMatrix[13];

  // Bottom clipping plane
  planes[enums.frustum.plane.BOTTOM][0] = comboMatrix[3] + comboMatrix[1];
  planes[enums.frustum.plane.BOTTOM][1] = comboMatrix[7] + comboMatrix[5];
  planes[enums.frustum.plane.BOTTOM][2] = comboMatrix[11] + comboMatrix[9];
  planes[enums.frustum.plane.BOTTOM][3] = comboMatrix[15] + comboMatrix[13];

  // Near clipping plane
  planes[enums.frustum.plane.NEAR][0] = comboMatrix[3] + comboMatrix[2];
  planes[enums.frustum.plane.NEAR][1] = comboMatrix[7] + comboMatrix[6];
  planes[enums.frustum.plane.NEAR][2] = comboMatrix[11] + comboMatrix[10];
  planes[enums.frustum.plane.NEAR][3] = comboMatrix[15] + comboMatrix[14];

  // Far clipping plane
  planes[enums.frustum.plane.FAR][0] = comboMatrix[3] - comboMatrix[2];
  planes[enums.frustum.plane.FAR][1] = comboMatrix[7] - comboMatrix[6];
  planes[enums.frustum.plane.FAR][2] = comboMatrix[11] - comboMatrix[10];
  planes[enums.frustum.plane.FAR][3] = comboMatrix[15] - comboMatrix[14];

  for (var i = 0; i < 6; ++i) {
    Plane.normalize(planes[i]);
  }

  //Sphere
  var fov = 1 / pMatrix[5];
  var near = -planes[enums.frustum.plane.NEAR][3];
  var far = planes[enums.frustum.plane.FAR][3];
  var view_length = far - near;
  var height = view_length * fov;
  var width = height;

  var P = [0, 0, near + view_length * 0.5];
  var Q = [width, height, near + view_length];
  var diff = vec3.subtract(P, Q);
  var diff_mag = vec3.length(diff);

  var look_v = [comboMatrix[3], comboMatrix[9], comboMatrix[10]];
  var look_mag = vec3.length(look_v);
  look_v = vec3.multiply(look_v, 1 / look_mag);

  var pos = [camera.position[0], camera.position[1], camera.position[2]];
  pos = vec3.add(pos, vec3.multiply(look_v, view_length * 0.5));
  pos = vec3.add(pos, vec3.multiply(look_v, 1));
  this.sphere = [pos[0], pos[1], pos[2], diff_mag];

}; //Frustum::extract

Frustum.prototype.contains_sphere = function(sphere) {
  var vec3 = CubicVR.vec3,
      planes = this._planes;

  for (var i = 0; i < 6; ++i) {
    var p = planes[i];
    var normal = [p[0], p[1], p[2]];
    var distance = vec3.dot(normal, [sphere[0],sphere[1],sphere[2]]) + p.d;
    this.last_in[i] = 1;

    //OUT
    if (distance < -sphere[3]) {
      return -1;
    }

    //INTERSECT
    if (Math.abs(distance) < sphere[3]) {
      return 0;
    }

  } //for
  //IN
  return 1;
}; //Frustum::contains_sphere

Frustum.prototype.draw_on_map = function(map_canvas, map_context) {
  var mhw = map_canvas.width/2;
  var mhh = map_canvas.height/2;
  map_context.save();
  var planes = this._planes;
  var important = [0, 1, 4, 5];
  for (var pi = 0, l = important.length; pi < l; ++pi) {
    var p = planes[important[pi]];
    map_context.strokeStyle = "#FF00FF";
    if (pi < this.last_in.length) {
      if (this.last_in[pi]) {
        map_context.strokeStyle = "#FFFF00";
      }
    } //if
    var x1 = -mhw;
    var y1 = (-p[3] - p[0] * x1) / p[2];
    var x2 = mhw;
    var y2 = (-p[3] - p[0] * x2) / p[2];
    map_context.moveTo(mhw + x1, mhh + y1);
    map_context.lineTo(mhw + x2, mhh + y2);
    map_context.stroke();
  } //for
  map_context.strokeStyle = "#0000FF";
  map_context.beginPath();
  map_context.arc(mhw + this.sphere[0], mhh + this.sphere[2], this.sphere[3], 0, Math.PI * 2, false);
  map_context.closePath();
  map_context.stroke();
  map_context.restore();
}; //Frustum::draw_on_map

Frustum.prototype.contains_box = function(bbox) {
  var total_in = 0;

  var points = [];
  points[0] = bbox[0];
  points[1] = [bbox[0][0], bbox[0][1], bbox[1][2]];
  points[2] = [bbox[0][0], bbox[1][1], bbox[0][2]];
  points[3] = [bbox[0][0], bbox[1][1], bbox[1][2]];
  points[4] = [bbox[1][0], bbox[0][1], bbox[0][2]];
  points[5] = [bbox[1][0], bbox[0][1], bbox[1][2]];
  points[6] = [bbox[1][0], bbox[1][1], bbox[0][2]];
  points[7] = bbox[1];

  var planes = this._planes;

  for (var i = 0; i < 6; ++i) {
    var in_count = 8;
    var point_in = 1;

    for (var j = 0; j < 8; ++j) {
      if (Plane.classifyPoint(planes[i], points[j]) === -1) {
        point_in = 0;
        --in_count;
      } //if
    } //for j
    this.last_in[i] = point_in;

    //OUT
    if (in_count === 0) {
      return -1;
    }

    total_in += point_in;
  } //for i
  //IN
  if (total_in === 6) {
    return 1;
  }

  return 0;
}; //Frustum::contains_box


  var exports = {
    Frustum: Frustum,
    Octree: Octree
  };

  return exports;
});