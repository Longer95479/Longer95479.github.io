---
layout: page
title: Organized by Tags
permalink: /tags/
content-type: eg
---

<!-- Bootstrap core CSS -->
<link crossorigin="anonymous" href="https://cdn.jsdelivr.net/npm/bootstrap@4.5.3/dist/css/bootstrap.min.css" integrity="sha384-TX8t27EcRE3e/ihU7zmQxVncDAy5uIKz4rEkgIXeMed4M0jlfIDPvg6uqKI2xXr2" rel="stylesheet">

<link rel="stylesheet" href="{{ "/css/main.css" | prepend: site.baseurl | replace: '//', '/' }}">

<style>
.category-content a {
    text-decoration: none;
    color: #4183c4;
}

.category-content a:hover {
    text-decoration: underline;
    color: #4183c4;
}
</style>

<div class="row">
    {% for tag in site.tags %}
    <div class="col-xl-4 col-lg-6 col-12" >
        <h3 id="{{ tag | first }}">{{ tag | first | capitalize }}</h3>
        <ul class="overflow-auto" id="tag-grid">
        {% for post in tag.last %}
            <li id="category-content"><a href="{{post.url}}">{{ post.title }}</a></li>
        {% endfor %}
        </ul>
    </div>
    {% endfor %}
    <br/>
    <br/>
</div>


