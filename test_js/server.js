const express = require('express');
const path = require('path'); // For serving static files
// const rclnodejs = require('rclnodejs');


const app = express();
const port = process.env.PORT || 8080; // Use environment variable or default port

// Serve static content from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Routes for your application (replace with your implementation)
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// ... other routes as needed

app.listen(port, () => {
  console.log(`Server listening on port ${port}`);
});
