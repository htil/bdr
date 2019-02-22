$(function() {
    "use strict";

    $('#bdr-form').on("submit",function(e) {
        e.preventDefault();
        var fname = $('#fname').val();
        var lname = $('#lname').val();
        var major = $('#major').val();
        console.log(fname, lname, major);
    
        clearLS();
        setLS('fname', fname);
        setLS('lname', lname);
        setLS('major', major);

        window.location.href = 'sim.html'
    });
    
    function setLS(id, val) {
        localStorage.setItem(id, val);
    }

    function clearLS() {
        localStorage.clear();
    }
});