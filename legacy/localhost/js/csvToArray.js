function csvToArray(_callback) {
  Papa.parse(document.getElementById('csv-file').files[0], {
    download: true,
    header: true,
    complete: function(results) {
        _callback(results);
      }
   });
}