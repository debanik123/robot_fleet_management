
var mapContainer = document.getElementById('map-container');
mapContainer.addEventListener('mousedown', onMousedown);
mapContainer.addEventListener('mousemove', onMousemove);
mapContainer.addEventListener('mouseup', onMouseup);

function onMousedown(event)
{
  var rect = mapContainer.getBoundingClientRect();
  const { clientX, clientY } = event.touches ? event.touches[0] : event;
	// start_point = {
	// 	x: clientX- rect.left,
	// 	y: clientY - rect.top
	// };

//   isDragging = true;
  console.log('mousedown');
//   init_start_point = null;
//   init_delta = null;
}


function onMousemove(event)
{
//   if (start_point === undefined) return;

	// const { clientX, clientY } = event.touches ? event.touches[0] : event;
	// delta = {
	// 	x: start_point.x - clientX,
	// 	y: start_point.y - clientY,
	// };
  // drawArrow();
  console.log('mouse move');

}


function onMouseup(event)
{
  console.log('mouseup');
  // send_nav2_goal_Message(start_point, delta);
  // drawArrow();
//   init_start_point = start_point;
//   init_delta = delta;
//   start_point = undefined;
//   delta = undefined;
}
