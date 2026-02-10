$(function() {
  var movies = new Bloodhound({
    datumTokenizer: Bloodhound.tokenizers.obj.whitespace('value'),
    queryTokenizer: Bloodhound.tokenizers.whitespace,
    remote: {
      url: '/autocomplete?query=%QUERY',
      wildcard: '%QUERY'
    }
  });

  $('#place-name').typeahead(null, {
    name: 'movies',
    limit: 10,
    display: 'value',
    hint: false,
    highlight: false,
    source: movies
  });

  $('#place-name').bind('typeahead:autocompleted', function(ev, suggestion) {
    ids[suggestion['value']] = suggestion['id']
  });

  $('#place-name').bind('typeahead:selected', function(ev, suggestion) {
    ids[suggestion['value']] = suggestion['id']
  });


  $('#end-name').typeahead(null, {
    name: 'end',
    limit: 10,
    display: 'value',
    hint: false,
    highlight: false,
    source: movies
  });

  $('#end-name').bind('typeahead:autocompleted', function(ev, suggestion) {
    ids_for_end[suggestion['value']] = suggestion['id']
  });

  $('#end-name').bind('typeahead:selected', function(ev, suggestion) {
    ids_for_end[suggestion['value']] = suggestion['id']
  });
});
