# ROS2とは？

## ROSとROS2の関係性
まず、ROSとはなにか？ということについてわかりやすい資料がこちら

<iframe src="https://docs.google.com/presentation/d/e/2PACX-1vQvX8nOOldvSrj9L7rbCSRcZuO4d4oCj_-KuVAo85D8urMcMmoIJIpqZDmme514_nvCcb4uaGTKdiut/embed?start=false&loop=false&delayms=3000" frameborder="0" width="960" height="569" allowfullscreen="true" mozallowfullscreen="true" webkitallowfullscreen="true"></iframe>

ロボットのソフトウェアは

- 密結合になりがち（ちゃんと動くシステムを作ろうとすると数万行になるのは当たり前）
- イベントドリブンになりがち（〇〇のセンサーに反応があったら〇〇する）という

性質があります。ROSを使うことによって、使いやすいロボットソフトウェアを作ることができます。  
具体的なメリットを上げておくと

- 同じハードウェアを使っている人がいた場合、開発期間が短縮できて本質的な研究活動に時間を使える
- 部品化ができるので「伝わる秘伝のタレ」化しにくい
- 部品のデバッグをしっかりしておけばデバッグ工数が削減できる

しかし、ROSにはその延長線上で開発を続けていては解決困難な問題があり、それを解決するべく1から再開発されたのがROS2です。  
ROSの最後のディストリビューションであるNoeticは[2025年にサポート終了が予告](http://wiki.ros.org/Distributions)されており、そろそろROS2に移行しておかないとせっかく作った開発資産が使えなくなってしまうリスクがあります。

## ROS2が作られた理由

#### ライセンス問題
ROS1時代はBSDライセンス等でコアライブラリが提供されてきました.
しかし、コアライブラリの一部にコピーレフトなライセンスに依存する可能性のあるものが含まれていたり、初期の開発過程がトレースされていなかったことによりライセンスに関して
不明瞭な部分が存在しました.
商用アプリケーション開発に於いてはこれは大きな法的問題に発生しうる可能性があり、1からROS2を開発する大きな要員の1つになりました.

ROS2からは[Apache 2.0ライセンス](https://licenses.opensource.jp/Apache-2.0/Apache-2.0.html)が採用され、それに準拠するようにコアライブラリ
の開発工程が管理されているため安心して商用利用することが可能です.

<details>
<summary>Apache 2.0 License</summary>

```

                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright [yyyy] [name of copyright owner]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
```

</details>

#### 単一障害点の排除
ROS1時代にはros masterというプロセスがrosparamの管理、新規ノードの発見やトピック間の接続という重要な仕事を担っていました.

**ros masterの役割の一例、TalkerとListenerノードの接続**
![](images/ros_master_0.png)
![](images/ros_master_1.png)
![](images/ros_master_2.png)
![](images/ros_master_3.png)
![](images/ros_master_4.png)
![](images/ros_master_5.png)

そのため、ROS1アプリケーションに於いてros masterが稼働中のシステムで落ちてしまうとシステム全体が機能不全になる可能性があります.
単一障害点の排除は長期間運用されるアプリケーションに於いては非常に重要な課題の1つです.
ros masterの挙動に関する日本語ドキュメントは[こちら](http://wiki.ros.org/ja/Master)にあります.

#### 効率的なデータ転送
ROS1時代に存在した[nodelet](http://wiki.ros.org/nodelet)という仕組みをご存知でしょうか？
nodeletは通常TCP/IPパケット通信により実現されるROSのトピック通信を共有ポインタを用いたゼロコピー通信に置き換えます.
この際、nodeletはnodelet_managerにロードされる共有ライブラリとして実装されます.
この仕組みをROS2向けに再設計したのが後述するROS2におけるコンポーネント指向であり、これを使用することで非常に高速にデータ通信が可能です.
![](images/nodelet.png)

#### Windows対応
ROS1はLinuxにかなり依存しており、Windowsで動かすにはWSLを使ったりと工夫が必要でした.
研究開発や、スタンドアローンなロボットであればROS1のLinux依存の強さも全く問題にはならなかったのですが、
商用アプリケーションを開発するときに一般のご家庭で動いているPCのOSとして圧倒的なシェアを持っているWindowsでアプリケーションが作れないのは問題になります.
そこでROS2からはWindowsにも対応し、(一応Macにも対応はしています.)Windows上でもロボットアプリケーション開発が可能になっています.
ROS1時代にはWindowsとROS1アプリケーションの通信は[rosbridge protocol](https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md)か
[nodejsのROSクライアント](https://github.com/RethinkRobotics-opensource/rosnodejs)くらいしかなかったので嬉しい人には嬉しい仕様かもしれません.

<blockquote class="embedly-card"><h4><a href="https://ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027">REP 2000 -- ROS 2 Releases and Target Platforms (ROS.org)</a></h4><p>Note The following applies to ROS 2 releases after Foxy. Prior to Foxy, releases were made more frequently but with shorter support due to the fact that many foundational parts of ROS 2 were still being heavily developed. New ROS 2 releases will be published in a time based fashion every 12 months.</p></blockquote>
<script async src="//cdn.embedly.com/widgets/platform.js" charset="UTF-8"></script>