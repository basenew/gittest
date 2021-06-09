function Joystick() {
  var manager = nipplejs.create({
    zone: document.getElementById('joystick_zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#999',
    size: 200
  });
  this.manager = manager;
  this.manager.options.lockX = true;
  this.manager.options.lockY = true;
  this.data = null;
  this.time = null;
}

Joystick.prototype.init = function() {
  this._on();
  return this;
}

Joystick.prototype._on = function() {
  var me = this;
  this.manager
    .on('start', function (evt, data) {
      me.time = setInterval(function() {
        me.onStart && me.onStart(me.data);
      }, 100);
    })
    .on('move', function (evt, data) {
      me.data = data;
    })
    .on('end', function (evt, data) {
      clearInterval(me.time);
      me.onEnd && me.onEnd(me.data);
      for (var i = 0; i<5; i++) {
          request_joystick_move(0, 0);
      }
    });
}

var obj_joystick = new Joystick()
obj_joystick.init()
obj_joystick.onStart = function(data) {
  if (data == null) return;
  if (data.direction) {
    var distance = data.distance/166;
    var direction_x = data.direction.x;
    var direction_y = data.direction.y;
    var degree = data.angle.degree;
    var degree_scale = 180 / Math.PI * 0.6;
    var degree_step = degree_scale / 90;

    if (data.direction.y == "down") {
      distance = -distance;
    }
    
    if (degree >= -90 && degree <= 90) {
      if (degree > 0) {
        distance = distance - distance * degree/90;
      } else {
        distance = distance + distance * degree/90;
      }
      degree = degree * degree_step;
    } else {
      if (degree > 0) {
        distance = distance - distance * (180 - degree)/90;
        degree = (180 - degree) * degree_step;
      } else {
        distance = distance + distance * (-180 - degree)/90;
        degree = (-180 - degree) * degree_step;
      }
    }
    
    var radian = Math.PI/180 * degree;
    request_joystick_move(distance, radian);
  }
}

document.getElementById("openjoystick").onclick = function() {
  if (document.getElementById("openjoystick").checked) {
    obj_joystick.manager.options.lockX = false;
    obj_joystick.manager.options.lockY = false;
  } else {
    obj_joystick.manager.options.lockX = true;
    obj_joystick.manager.options.lockY = true;
  }
}