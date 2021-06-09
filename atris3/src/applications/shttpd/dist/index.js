function $(id){
  return typeof id === 'string' ? document.getElementById(id):id;
}

window.onload = function() {
  var titles = $('tab-header').getElementsByTagName('li');
  var divs = $('tab-content').getElementsByClassName('dom');
  if((titles.length <= 0) || (titles.length != divs.length)) return;
  for(var i=0; i<titles.length; i++){
    var li = titles[i];
    li.id = i;
    li.onclick = function(){
      for(var j=0; j<titles.length; j++){
        titles[j].className = '';
        divs[j].style.display = 'none';
      }
      this.className = 'selected';
      divs[this.id].style.display = 'block';
    }
  }
}

document.onkeydown = function (){
  var e = window.event || arguments[0];
  if(e.keyCode == 123) { //F12
    return false;
  } else if(e.keyCode == 255) { //360
    return false;
  } else if ((e.ctrlKey) && (e.shiftKey) && (e.keyCode == 73)) { //Ctrl+Shift+I
    return false;
  } else if ((e.shiftKey) && (e.keyCode == 121)) { //Shift+F10
    return false;
  } else if ((e.ctrlKey) && (e.keyCode == 85)) { //Ctrl+U
    return false;
  }
};

document.oncontextmenu = function (){
  return false;
}
