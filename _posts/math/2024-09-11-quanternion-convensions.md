---
layout: post
title:  "Quaternion Conventions"
date:   2024-09-11 17:21:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

<table>
	<tbody>
		<tr>
			<th> </th>
			<th>Quaternion type</th>
			<th>Hamilton</th>
			<th>$JPL$</th>
		</tr>
		<tr>
			<td>1</td>
			<td>Components s order</td>
			<td>$( q_w$ , $\mathbf{q} _v)$</td>
			<td>$\left(\mathbf{q}_v\:,\:\right.$ $q_w)$</td>
		</tr>
		<tr>
			<td rowspan="1">2</td>
			<td>$\operatorname{Algebra}$ Handedness</td>
			<td>$ij=k$ Right- handed</td>
			<td>$ij=$ $=-k$ Left- handed</td>
		</tr>
		<tr>
			<td>3</td>
			<td>Function</td>
			<td>Passive</td>
			<td>$\mathrm{P}_{\xi}$ assive</td>
		</tr>
		<tr>
			<td> </td>
			<td>Right- to- left products mean</td>
			<td>Local- to- Global</td>
			<td>Global-to-Local</td>
		</tr>
		<tr>
			<td rowspan="1">4</td>
			<td>Default operation </td>
			<td>$\mathbf{X}_{G}$ $\mathbf{X}_{\mathcal{L}}$ 水 = $\mathbf{q}$ $\mathbf{q}$</td>
			<td>$\mathbf{X}_{C}$ 三 T $\mathbf{X}_{G}$</td>
		</tr>
	</tbody>
</table>

