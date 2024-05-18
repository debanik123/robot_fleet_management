const express = require('express');
const path = require('path'); // For serving static files
// const rclnodejs = require('rclnodejs');
const { initializePublisher, publishVelocity } = require('./public/js/cmd_vel_publisher');
initializePublisher();

const app = express();
const port = process.env.PORT || 8000; // Use environment variable or default port

// Serve static content from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

app.post('/cmd_vel', express.json(), (req, res) => {
  // Extract velocity data from the request body:
  const { linearX, linearY, angularZ } = req.body;
  if (
    typeof linearX !== 'number' ||
    typeof linearY !== 'number' ||
    typeof angularZ !== 'number'
  ) {
    res.status(400).send('Invalid velocity data. Please provide numbers for linearX, linearY, and angularZ.');
    return; // Exit the route handler if validation fails
  }

  // Publish velocity using cmd_vel_publisher:
  publishVelocity(linearX, linearY, angularZ);
  // console.log(linearX, linearY, angularZ);

  // Send a success response:
  res.sendStatus(200);
});

// Routes for your application (replace with your implementation)
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// ... other routes as needed

app.listen(port, () => {
  console.log(`Server listening on port ${port}`);
});
