var ids = {}, ids_for_end = {};

$(function() {
  // TODO: update view location to be UIC-area
    mymap = L.map('mapid').setView([41.871822, -87.651280], 17);

    var greenIcon = new L.Icon({
      iconUrl: 'https://cdn.rawgit.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
      shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
      iconSize: [25, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowSize: [41, 41]
    });

    // TODO: replace this with our own, not vignesh's lmao
    L.tileLayer('https://api.mapbox.com/styles/v1/vigneshv59/cjs8ac0jc1lsb1fo05dxnke1r/tiles/256/{z}/{x}/{y}@2x?access_token=pk.eyJ1IjoidmlnbmVzaHY1OSIsImEiOiJjanM4OW40emwwZDloNGJtam9zNjczM2l1In0.EcwQfh4bxp5HCB6sqy0uhw', {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
        maxZoom: 19,
        id: 'mapbox.streets',
        accessToken: 'your.mapbox.access.token'
    }).addTo(mymap);

    var last_active = $("#place-name");
    var markers, path, circle;

    function update(name, id) {
        if (last_active[0].id === "place-name") {
            ids[name] = id;
        }
        else {
            ids_for_end[name] = id;
        }
        last_active.val(name);
    }

    function clear() {
       if (path) {
          mymap.removeLayer(path);
          path = null;
       }
       if (circle) {
          mymap.removeLayer(circle);
          circle = null;
       }
       if (markers) {
          markers.clearLayers()
          markers = null;
       }
    }

    $("#search").on('click', function() {
        $("#search").addClass('active');
        $("#directions").removeClass('active');
        $("#end-name").attr('hidden', true);
        $("#all").removeAttr('hidden');
        $("#nearby").removeAttr('hidden');
        $("#findpath").attr('hidden', true);
        last_active = $("#place-name");
    });

    $("#directions").on('click', function() {
        $("#directions").addClass('active');
        $("#search").removeClass('active');
        $("#end-name").removeAttr('hidden');
        $("#all").attr('hidden', true);
        $("#nearby").attr('hidden', true);
        $("#findpath").removeAttr('hidden');
    });


    $("#place-name").on('focus', function() {
        last_active = $("#place-name");
    });

    $("#end-name").on('focus', function() {
        last_active = $("#end-name");
    });

    $("#findpath").click(function() {
       clear();
       $.get("/pathfinder", {'start-id': ids[$("#place-name").val()], 'start': $("#place-name").val(), 'end-id': ids_for_end[$("#end-name").val()],'end': $("#end-name").val()}, function(data) {
            // data = data.substring(1, data.length - 1).split(",");
            var start = data[0];
            var end = data[data.length - 1];
            data = data.slice(1, data.length - 1);
            var ms = [];
            $.each(data, function(i, val) {
                var varr = val.split("::");
                var lat = varr[0];
                var lon = varr[1];

                var m = new L.LatLng(parseFloat(lat), parseFloat(lon));
                ms.push(m);
            })

            path = new L.Polyline(ms, {
              color: 'blue',
              weight: 3,
              opacity: 0.5,
              smoothFactor: 1
            });
            path.addTo(mymap);
        });
    });

    $("#nearby").click(function() {
        clear();
        var pname = $("#place-name").val();
        pname = pname.replace(/ \(.*\)$/, "")

       $.get("/nearby", {'id': ids[$("#place-name").val()], 'name': pname, 'distance': 200}, function(data) {
            // data = data.substring(1, data.length - 1).split(",")
            var ms = [];
            var varr = data[0].split("::");
            var lat = varr[0];
            var lon = varr[1];
            var m = L.marker([parseFloat(lat), parseFloat(lon)], {icon: greenIcon})
            if (varr[2] != "null") {
                m.bindPopup(varr[2]).on('click', function() {
                   update(varr[2], varr[3]);
                });
            }

            ms.push(m)
            mymap.setView([parseFloat(lat), parseFloat(lon)], 18);
            data = data.slice(1);
            $.each(data, function(i, val) {
                var varr = val.split("::")
                var lat = varr[0]
                var lon = varr[1]

                var m = L.marker([parseFloat(lat), parseFloat(lon)])
                if (varr[2] != "null") {
                    m.bindPopup(varr[2]).on('click', function() {
                        update(varr[2], varr[3]);
                    });
                }

                ms.push(m)
            })

            circle = new L.circle([lat, lon], 60);
            circle.addTo(mymap);

            markers = L.layerGroup(ms).addTo(mymap);
       })
    })

    $("#all").click(function() {
       clear();
       var pname = $("#place-name").val();
       pname = pname.replace(/ \(.*\)$/, "")
       $.get("/byname", {'query': pname}, function(data) {
            // data = data.substring(1, data.length - 1).split(",")
            var ms = [];
            mymap.setZoom(16);
            $.each(data, function(i, val) {
                var varr = val.split("::")
                var lat = varr[0]
                var lon = varr[1]

                var m = L.marker([parseFloat(lat), parseFloat(lon)])
                if (varr[2] != "null") {
                    m.bindPopup(varr[2]).on('click', function() {
                        update(varr[2], varr[3]);
                    });
                }

                ms.push(m)
            })

            markers = L.layerGroup(ms).addTo(mymap);
       })
    });

    var _dblClickTimer = null;

    mymap.addEventListener('click', function(ev) {
       lat = ev.latlng.lat;
       lon = ev.latlng.lng;
         if (_dblClickTimer !== null) {
           return;
         }
         _dblClickTimer = setTimeout(() => {
           $.getJSON("/nearest", {'lat': lat, 'lon': lon}, function(data) {
              clear();
              var ms = [];
              var m = L.marker([data.lat, data.lon], {icon: greenIcon});
              update(data.name, data.id);
              if (data.name != "null") {
                  m.bindPopup(data.name).on('click', function() {
                    update(data.name, data.id);
                  })
              }
              ms.push(m)
              markers = L.layerGroup(ms).addTo(mymap);
           });

           _dblClickTimer = null;
         }, 200);
    }).on("dblclick", function() {
        clearTimeout(_dblClickTimer);
        _dblClickTimer = null;

        // real 'dblclick' handler here (if any). Do not add anything to just have the default zoom behavior
    });
})
